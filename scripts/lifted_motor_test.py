#!/usr/bin/env python3
"""Bounded supported-wheels test. Requires explicit physical setup confirmation.

Uses the independent base guard, never the public /cmd_vel topic. Ends with a
SIGKILL test of guard/base lifetime and read-only OpenCR stop observation.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def producer(path):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    rclpy.init();node=Node('lifted_test_command_source')
    pub=node.create_publisher(Twist,'/radar/cmd_vel_safe',1)
    end=time.monotonic()+90
    try:
        while time.monotonic()<end:
            message=Twist()
            try:
                command=json.loads(path.read_text())
                if time.monotonic()<command['deadline']:
                    v,w=command['linear'],command['angular']
                    if all(math.isfinite(x) for x in (v,w)) and 0<=v<=.04 and abs(w)<=.2:
                        message.linear.x=v;message.angular.z=w
            except (OSError,KeyError,ValueError):pass
            pub.publish(message);rclpy.spin_once(node,timeout_sec=.001);time.sleep(.049)
    finally:node.destroy_node();rclpy.shutdown()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wheels-lifted-confirmed',action='store_true')
    ap.add_argument('--output',type=Path)
    ap.add_argument('--producer',type=Path)
    args=ap.parse_args()
    if not args.wheels_lifted_confirmed:ap.error('Confirm secured support, both wheels clear, power and operator present')
    if args.producer:return producer(args.producer)
    if args.output is None:ap.error('--output required')
    out=args.output;out.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool,String
    rclpy.init();node=Node('lifted_motor_evidence')
    states=[];joints=[];odom=[];commands=[];events=[];enable=False;source=None
    pub=node.create_publisher(Bool,'/radar/enable',1)
    subs=[node.create_subscription(String,'/base/watchdog_status',lambda m:states.append((time.monotonic(),json.loads(m.data))),20),
        node.create_subscription(JointState,'/joint_states',lambda m:joints.append(dict(t=time.monotonic(),position=list(m.position),velocity=list(m.velocity))),20),
        node.create_subscription(Odometry,'/odom',lambda m:odom.append(dict(t=time.monotonic(),x=m.pose.pose.position.x,y=m.pose.pose.position.y,linear=m.twist.twist.linear.x,angular=m.twist.twist.angular.z)),20),
        node.create_subscription(Twist,'/base/driver_cmd_vel',lambda m:commands.append(dict(t=time.monotonic(),linear=m.linear.x,angular=m.angular.z)),20)]
    hard_end=time.monotonic()+90
    command_file=out/'command.json';result=dict(passed=False,physical_setup='wheels lifted; chassis stationary; operator-confirmed support/top plate/12V supply',floor_motion=False)
    def event(name,**data):
        row=dict(event=name,unix=time.time(),monotonic=time.monotonic(),**data);events.append(row)
        with (out/'events.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
        print(name,flush=True);return row['monotonic']
    def spin(seconds):
        end=time.monotonic()+seconds;next_enable=0
        while time.monotonic()<end:
            if time.monotonic()>hard_end:raise RuntimeError('Absolute test deadline exceeded')
            if time.monotonic()>=next_enable:pub.publish(Bool(data=enable));next_enable=time.monotonic()+.05
            rclpy.spin_once(node,timeout_sec=.005)
    def set_command(v,w,seconds):
        temp=out/'command.new';temp.write_text(json.dumps(dict(linear=v,angular=w,deadline=time.monotonic()+seconds)));temp.replace(command_file)
    def start_source():
        set_command(0.,0.,3)
        log=(out/('producer_'+str(len(events))+'.log')).open('w')
        process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--wheels-lifted-confirmed','--producer',str(command_file)],stdout=log,stderr=log,start_new_session=True)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            spin(.1)
            if states and states[-1][1]['command_age'] is not None and states[-1][1]['command_age']<.2:return process
        raise RuntimeError('Command source did not connect')
    def phase(name,v,w,duration):
        start=event(name,linear=v,angular=w);set_command(v,w,duration+.15);spin(duration)
        return start
    def stop_delay(start):
        rr=[r for r in joints if r['t']>=start]
        for i in range(len(rr)-2):
            if all(len(r['velocity'])==2 and max(abs(v) for v in r['velocity'])<=.002 for r in rr[i:i+3]):return rr[i]['t']-start
        return None
    def active():
        rr=[r for r in joints if r['t']>=time.monotonic()-.25]
        return bool(rr) and any(max(abs(v) for v in r['velocity'])>.01 for r in rr)
    try:
        spin(3)
        if not states or not states[-1][1]['actuate']:raise RuntimeError('Requires explicitly actuating guard on supported wheels')
        if len(node.get_publishers_info_by_topic('/radar/cmd_vel_safe')):raise RuntimeError('Stop reactive controller before bounded motor test')
        base_pid=states[-1][1]['base_pid']
        status=Path(f'/proc/{base_pid}/status').read_text();guard_pid=int(next(l.split()[1] for l in status.splitlines() if l.startswith('PPid:')))
        assert b'turtlebot3_ros' in Path(f'/proc/{base_pid}/cmdline').read_bytes()
        assert b'base_watchdog' in Path(f'/proc/{guard_pid}/cmdline').read_bytes()
        result.update(base_pid=base_pid,guard_pid=guard_pid)
        source=start_source();enable=True;spin(.3)
        begin=phase('forward_pulse',.04,0.,2)
        rr=[r for r in joints if r['t']>begin+.5]
        result['forward_both_encoders_positive']=bool(rr) and all(v>0 for v in rr[-1]['velocity'])
        phase('forward_zero',0.,0.,1)
        turn=phase('left_turn_pulse',0.,.2,2)
        rr=[r for r in joints if r['t']>turn+.5]
        result['left_turn_encoder_signs']=bool(rr) and rr[-1]['velocity'][0]<0<rr[-1]['velocity'][1]
        stop=phase('explicit_zero',0.,0.,1)
        result['explicit_zero_stop_s']=stop_delay(stop)
        if result['explicit_zero_stop_s'] is None or result['explicit_zero_stop_s']>.7:raise RuntimeError('Explicit zero did not stop supported wheels')
        phase('disable_test_motion',.04,0.,.8);assert active()
        disable=event('enable_false');enable=False;spin(1)
        result['disable_stop_s']=stop_delay(disable)
        if result['disable_stop_s'] is None or result['disable_stop_s']>.7:raise RuntimeError('Enable release did not stop supported wheels')
        set_command(0.,0.,1);spin(.2);enable=True
        phase('producer_death_motion',.04,0.,.8);assert active()
        killed=event('command_producer_sigkill',pid=source.pid);source.kill();source.wait();source=None
        # Keep the independent enable lease alive to isolate command freshness.
        spin(1.2);result['producer_death_stop_s']=stop_delay(killed)
        if result['producer_death_stop_s'] is None or result['producer_death_stop_s']>.7:raise RuntimeError('Command producer death did not stop wheels')
        result['producer_death_guard_reason']=states[-1][1]['reason']
        enable=False;spin(.3)
        while node.get_publishers_info_by_topic('/radar/cmd_vel_safe'):spin(.1)
        spin(.5);source=start_source();enable=True;spin(.3)
        phase('guard_death_motion',.04,0.,.8);assert active()
        killed=event('base_watchdog_sigkill',pid=guard_pid,base_pid=base_pid)
        os.kill(guard_pid,signal.SIGKILL);enable=False
        # Kernel kills serial-owning child. Read-only monitoring sends no heartbeat.
        deadline=time.monotonic()+1
        port_path=Path('/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00')
        while time.monotonic()<deadline:
            if subprocess.run(['fuser',str(port_path.resolve())],capture_output=True).returncode==1:break
            spin(.01)
        else:raise RuntimeError('Base child retained serial port after parent death')
        from dynamixel_sdk import PortHandler,PacketHandler,COMM_SUCCESS
        port=PortHandler(str(port_path));packet=PacketHandler(2.)
        assert port.openPort() and port.setBaudRate(1000000)
        raw=[];emergency=False
        try:
            while time.monotonic()<killed+1.3:
                data,comm,error=packet.readTxRx(port,200,128,16)
                if comm==COMM_SUCCESS and not error:
                    import struct
                    values=struct.unpack('<4i',bytes(data));raw.append(dict(t=time.monotonic(),velocity=list(values[:2]),position=list(values[2:])))
                    if time.monotonic()>killed+1 and max(abs(v) for v in values[:2])>2:
                        packet.writeTxRx(port,200,150,24,[0]*24);packet.write1ByteTxRx(port,200,149,0);emergency=True
                spin(.015)
            heartbeat,_,_=packet.read1ByteTxRx(port,200,19)
            command,_,_=packet.read4ByteTxRx(port,200,150)
            result.update(heartbeat_after_guard_death=heartbeat,latched_linear_register_after_guard_death=command,emergency_stop_required=emergency)
            packet.writeTxRx(port,200,150,24,[0]*24)
        finally:port.closePort()
        result['opencr_raw_samples']=raw
        result['opencr_stop_after_guard_death_s']=next((r['t']-killed for i,r in enumerate(raw[:-2]) if all(max(abs(v) for v in s['velocity'])<=2 for s in raw[i:i+3])),None)
        result['base_child_killed_by_kernel']=not Path(f'/proc/{base_pid}').exists() or Path(f'/proc/{base_pid}/stat').read_text().split()[2]=='Z'
        result['passed']=all(result[k] for k in ['forward_both_encoders_positive','left_turn_encoder_signs','base_child_killed_by_kernel']) and all(result[k] is not None and result[k]<.7 for k in ['explicit_zero_stop_s','disable_stop_s','producer_death_stop_s']) and result['opencr_stop_after_guard_death_s'] is not None and result['opencr_stop_after_guard_death_s']<.9 and not emergency
        event('lifted_test_complete',passed=result['passed'])
    finally:
        enable=False
        for _ in range(5):pub.publish(Bool(data=False));rclpy.spin_once(node,timeout_sec=.01);time.sleep(.03)
        if source and source.poll() is None:source.kill();source.wait()
        if not result['passed'] and 'guard_pid' in result:
            identity=Path(f"/proc/{result['guard_pid']}/cmdline")
            if identity.exists() and b'base_watchdog' in identity.read_bytes():os.kill(result['guard_pid'],signal.SIGKILL)
        (out/'measurements.json').write_text(json.dumps(dict(joints=joints,odom=odom,base_commands=commands,base_status=states),indent=2)+'\n')
        (out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
