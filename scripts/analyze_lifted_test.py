#!/usr/bin/env python3
"""Reproduce supported-wheel telemetry summary and plots; no ROS or hardware access."""
import argparse
import json
import math
from pathlib import Path
import statistics


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory',type=Path)
    args=ap.parse_args();root=args.directory
    d=json.loads((root/'measurements.json').read_text())
    metrics=json.loads((root/'metrics.json').read_text())
    events=[json.loads(l) for l in (root/'events.jsonl').read_text().splitlines()]
    summary={k:v for k,v in metrics.items() if k!='opencr_raw_samples'}
    summary['method']='Wheel-only odometry on support; chassis stationary. NOT floor accuracy or stopping distance.'
    summary['velocity_units']='Installed JointState velocities empirically match wheel-surface m/s; positions are rad.'
    summary['stop_criterion']='Three consecutive wheel samples <=0.002 m/s (ROS) or <=2 raw motor units (OpenCR); time of first sample.'
    summary['pulses']={}
    for name in ['forward_pulse','left_turn_pulse']:
        event=next(e for e in events if e['event']==name);t=event['monotonic']
        jj=[r for r in d['joints'] if t<=r['t']<=t+2]
        oo=[r for r in d['odom'] if t+.5<=r['t']<=t+1.8]
        delta=[jj[-1]['position'][i]-jj[0]['position'][i] for i in range(2)]
        summary['pulses'][name]=dict(wheel_angle_change_rad=delta,
            encoder_distance_m=.033*sum(delta)/2,
            encoder_yaw_rad=.033*(delta[1]-delta[0])/.287,
            steady_odom_linear_m_s=statistics.median(r['linear'] for r in oo),
            steady_odom_angular_rad_s=statistics.median(r['angular'] for r in oo),
            steady_left_surface_m_s=statistics.median(r['velocity'][0] for r in jj if t+.5<=r['t']<=t+1.8),
            steady_right_surface_m_s=statistics.median(r['velocity'][1] for r in jj if t+.5<=r['t']<=t+1.8))
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    zero=events[0]['monotonic'];fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for i,label in enumerate(['left','right']):axes[0].plot([r['t']-zero for r in d['joints']],[r['velocity'][i] for r in d['joints']],label=label)
    axes[0].set_ylabel('Wheel surface m/s');axes[0].legend()
    for field,label in [('linear','m/s'),('angular','rad/s')]:axes[1].plot([r['t']-zero for r in d['base_commands']],[r[field] for r in d['base_commands']],label=label)
    axes[1].set_ylabel('Guard commands');axes[1].legend()
    raw=metrics.get('opencr_raw_samples',[])
    for i,label in enumerate(['left','right']):axes[2].plot([r['t']-zero for r in raw],[r['velocity'][i] for r in raw],'.-',label=label)
    axes[2].set_ylabel('Raw motor velocity');axes[2].set_xlabel('Seconds since first forward pulse');axes[2].legend()
    for event in events:
        if event['event'] in ['explicit_zero','enable_false','command_producer_sigkill','base_watchdog_sigkill']:
            for ax in axes:ax.axvline(event['monotonic']-zero,color='gray',alpha=.4)
            axes[0].annotate(event['event'].replace('_',' '),(event['monotonic']-zero,.045),rotation=75,fontsize=7)
    fig.suptitle('Supported wheels: physical telemetry and stop tests (no floor motion)')
    fig.tight_layout();fig.savefig(root/'motor_stop_tests.png',dpi=160);plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
