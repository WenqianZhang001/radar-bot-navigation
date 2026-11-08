"""Measure real map expiry against replayed fresh evidence and bag map values."""
import argparse
import datetime
import json
import math
import pathlib
import sqlite3

from radar_navigation.core import Filter, Pose
from radar_navigation.protocol import Frame
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid

parser=argparse.ArgumentParser()
parser.add_argument('directory',type=pathlib.Path)
args=parser.parse_args();out=args.directory
rows=[json.loads(l) for l in (out/'observations.jsonl').read_text().splitlines()]
events={r['event']:r for r in map(json.loads,(out/'events.jsonl').read_text().splitlines())}
metrics=json.loads((out/'spatial_metrics.json').read_text())
experiment=json.loads((out/'experiment.json').read_text())
indices=metrics['target_exclusive_map_cells']['indices']
x,y,z=experiment['mount']['translation_m'];roll,pitch,yaw=experiment['mount']['rpy_deg']
assert roll==pitch==0,'This deployed perception revision requires planar mounting'
pose=Pose(x,y,math.radians(yaw),z)
filter_=Filter()
last_evidence={}
processed_stamps={r['stamp_ns'] for r in rows if r['kind']=='processed'}
for r in rows:
    if r['kind']!='metadata':continue
    frame=Frame(**r['data'])
    if frame.stamp_ns not in processed_stamps:continue
    filter_.update(frame,pose)
    for p in filter_.fresh:
        cx=math.floor((p[0]+10)/.1);cy=math.floor((p[1]+10)/.1)
        i=cy*200+cx
        if i in indices and frame.stamp_ns/1e9<=events['clear_ack']['unix']:
            last_evidence[i]=dict(stamp_ns=frame.stamp_ns,point_base_footprint=list(p),metadata_receipt_unix=r['unix'])

db=sqlite3.connect('file:'+str(out/'experiment_bag/experiment_bag_0.db3')+'?mode=ro',uri=True)
topic=db.execute("SELECT id FROM topics WHERE name='/radar_front/occupancy'").fetchone()[0]
start=int((events['clear_request']['unix']-5)*1e9)
end=int((events['cleared_end']['unix']+1)*1e9)
maps=[]
for receipt,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',(topic,start,end)):
    m=deserialize_message(data,OccupancyGrid)
    stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
    maps.append(dict(receipt_ns=receipt,stamp_ns=stamp,values={i:int(m.data[i]) for i in indices}))
db.close()
result=dict(method='Replay accepted metadata through unchanged Filter to identify fresh (not accumulated) evidence; read actual bag OccupancyGrid cell values and header times.',physical_removal_time='Unknown exact second, bounded by request and acknowledgement; evidence cessation is an observed proxy, not exact operator action.',cells=[])
for i in indices:
    last=last_evidence.get(i)
    if not last:continue
    expiry=next((m for m in maps if m['stamp_ns']>last['stamp_ns'] and m['values'][i]==-1),None)
    recurrence=[m for m in maps if expiry and m['stamp_ns']>expiry['stamp_ns'] and m['values'][i]>=65]
    result['cells'].append(dict(index=i,center_base_footprint_m=[-10+(i%200+.5)*.1,-10+(i//200+.5)*.1],last_fresh_evidence=last,first_unknown_map=expiry,expiry_after_last_fresh_evidence_s=(expiry['stamp_ns']-last['stamp_ns'])/1e9 if expiry else None,occupied_recurrences_after_expiry=len(recurrence),expiry_relative_to_clear_ack_s=expiry['stamp_ns']/1e9-events['clear_ack']['unix'] if expiry else None))
if last_evidence:
    latest=max(e['stamp_ns'] for e in last_evidence.values())
    all_unknown=next((m for m in maps if m['stamp_ns']>latest and all(v==-1 for v in m['values'].values())),None)
    result['all_selected_unknown_after_latest_evidence_s']=(all_unknown['stamp_ns']-latest)/1e9 if all_unknown else None
    result['first_all_unknown_map']=all_unknown
    result['last_target_area_evidence_utc']=datetime.datetime.fromtimestamp(latest/1e9,datetime.timezone.utc).isoformat()
    result['first_all_unknown_utc']=datetime.datetime.fromtimestamp(all_unknown['stamp_ns']/1e9,datetime.timezone.utc).isoformat() if all_unknown else None
(out/'expiry_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
if last_evidence:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,4))
    for i in indices:
        ax.step([(m['stamp_ns']-latest)/1e9 for m in maps],[m['values'][i] for m in maps],where='post',label=f'cell {i}')
        ax.axvline((last_evidence[i]['stamp_ns']-latest)/1e9,linestyle=':',alpha=.5)
    ax.axvspan(1.5,1.7,color='gray',alpha=.2,label='1.5 s TTL + up to 0.2 s map timer')
    ax.set(xlim=(-2,4),ylim=(-5,103),xlabel='seconds from last fresh evidence in either target-area cell',ylabel='recorded occupancy value',title='Physical removal with acquisition continuing: evidence expires to unknown')
    ax.set_yticks([-1,70,98]);ax.set_yticklabels(['unknown (-1)','70','98']);ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'expiry_detail.png',dpi=170);plt.close(fig)
print(json.dumps(result,indent=2))
