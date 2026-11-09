#!/usr/bin/env python3
"""Two bounded visual direction pulses; ONLY with secured wheels lifted."""
import argparse,json,time
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wheels-lifted-confirmed',action='store_true')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--sequence',choices=['directions','isolated'],default='directions')
    ap.add_argument('--start-at-unix',type=float,default=0.)
    args=ap.parse_args()
    if not args.wheels_lifted_confirmed:ap.error('Supported wheels and present observer required')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool,String
    from sensor_msgs.msg import JointState
    rclpy.init();n=Node('visual_lifted_pulses');states=[];joints=[];events=[]
    command=n.create_publisher(Twist,'/radar/cmd_vel_safe',1)
    enable=n.create_publisher(Bool,'/radar/enable',1)
    subs=[n.create_subscription(String,'/base/watchdog_status',lambda m:states.append(json.loads(m.data)),10),n.create_subscription(JointState,'/joint_states',lambda m:joints.append(dict(t=time.monotonic(),position=list(m.position),velocity=list(m.velocity))),20)]
    def phase(name,v,w,seconds,armed):
        events.append(dict(name=name,unix=time.time(),monotonic=time.monotonic(),v=v,w=w));print(name,flush=True)
        end=time.monotonic()+seconds;next_pub=0
        while time.monotonic()<end:
            if time.monotonic()>=next_pub:
                msg=Twist();msg.linear.x=v;msg.angular.z=w;command.publish(msg);enable.publish(Bool(data=armed));next_pub=time.monotonic()+.05
            rclpy.spin_once(n,timeout_sec=.005)
    try:
        phase('prepare_zero',0.,0.,3,False)
        assert states and states[-1]['actuate'],'Requires supported-wheel actuating guard'
        assert len(n.get_publishers_info_by_topic('/radar/cmd_vel_safe'))==1,'Competing command source'
        if args.start_at_unix:
            assert 0<args.start_at_unix-time.time()<60,'Scheduled start must be within next minute'
            while time.time()<args.start_at_unix:phase('waiting_zero',0.,0.,min(.2,args.start_at_unix-time.time()),False)
        if args.sequence=='isolated':
            # v +/- w*b/2: stock b=.287; firmware quantizes w to .01 rad/s.
            # ~0 and +.04 m/s at the native left/right channels, then swapped.
            phase('A_native_right_channel',.02,.14,2,True)
            phase('stop_three_seconds',0.,0.,3,False)
            phase('B_native_left_channel',.02,-.14,2,True)
        else:
            phase('forward_two_seconds',.04,0.,2,True)
            phase('stop_three_seconds',0.,0.,3,False)
            phase('left_turn_two_seconds',0.,.2,2,True)
        phase('final_stop',0.,0.,1,False)
    finally:
        for _ in range(10):command.publish(Twist());enable.publish(Bool(data=False));rclpy.spin_once(n,timeout_sec=.01)
        (args.output/'measurements.json').write_text(json.dumps(dict(events=events,joints=joints,states=states),indent=2)+'\n')
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
