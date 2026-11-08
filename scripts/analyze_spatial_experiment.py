"""Analyze phased stationary radar observations; retain clutter and survey limits."""
import argparse
import collections
import json
import math
import pathlib
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('directory',type=pathlib.Path)
args=parser.parse_args()
out=args.directory
experiment=json.loads((out/'experiment.json').read_text())
rows=[json.loads(l) for l in (out/'observations.jsonl').read_text().splitlines()]
events=[json.loads(l) for l in (out/'events.jsonl').read_text().splitlines()]
times={e['event']:e for e in events}
translation=np.asarray(experiment['mount']['translation_m'])
roll,pitch,yaw=np.radians(experiment['mount']['rpy_deg'])
rx=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
ry=np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
rotation=rz@ry@rx
target=experiment.get('reflector')
bounds=None
if target:
    center=np.asarray(target['center_m'])
    width,height,depth=target['width_m'],target['height_m'],target['depth_m']
    bounds=np.array([[center[0]-depth,center[0]],[center[1]-width/2,center[1]+width/2],[center[2]-height/2,center[2]+height/2]])
    padding=np.array([.12,.20,.20])

def cloud_points(row):
    points=np.asarray([p[:3] for p in row['points']],dtype=float).reshape((-1,3))
    return points@rotation.T+translation if row['kind']=='source' else points

def roi(points):
    if bounds is None:return np.zeros(len(points),dtype=bool)
    return np.all((points>=bounds[:,0]-padding)&(points<=bounds[:,1]+padding),axis=1)

def percentile(values,q):return float(np.percentile(values,q)) if len(values) else None
def rate(rr):return (len(rr)-1)/(rr[-1]['monotonic']-rr[0]['monotonic']) if len(rr)>1 else None
phases={}
metrics=dict(experiment=experiment,phase_metrics={},notes=[
    'Earlier recordings may contain the reflector; only the newly timestamped absent baseline is used.',
    'No motion commanded. Mount translation is operator-surveyed +/- 1-2 cm; angle uncertainty is unquantified.',
    'Concave-reflector envelope is a conservative box, not a point scatterer or exact surface model.',
    'ROI padding is 0.12 m in x and 0.20 m in y/z; not an accuracy acceptance tolerance.',
    'Voxel association is descriptive baseline contrast, not proof that every return is from the reflector.',
    'Physical removal occurs between request and acknowledgement; chat receipt is not an exact action timestamp.',
    'Present condition adds foil, boxes and attached marker/cardboard. Post-removal retains boxes only. Final cleared condition removes all added objects.'])
for phase in ['baseline','reflector','post_removal','cleared']:
    if phase+'_begin' not in times or phase+'_end' not in times:continue
    start,end=times[phase+'_begin']['monotonic'],times[phase+'_end']['monotonic']
    selected=[r for r in rows if start<=r['monotonic']<=end]
    source=[r for r in selected if r['kind']=='source']
    processed=[r for r in selected if r['kind']=='processed']
    maps=[r for r in selected if r['kind']=='occupancy']
    points=np.concatenate([cloud_points(r) for r in source]) if source else np.empty((0,3))
    cells=collections.Counter(i for r in maps for i,v in r['occupied'])
    voxel=collections.Counter()
    for r in source:
        pp=cloud_points(r)
        for key in set(tuple(np.floor(p/.1).astype(int)) for p in pp[roi(pp)]):voxel[key]+=1
    pm=dict(begin_unix=times[phase+'_begin']['unix'],end_unix=times[phase+'_end']['unix'],duration_s=end-start,source_frames=len(source),processed_frames=len(processed),source_hz=rate(source),processed_hz=rate(processed),map_hz=rate(maps),empty_source_frames=sum(not r['points'] for r in source),source_points=len(points),occupied_cells_median=statistics.median([len(r['occupied']) for r in maps]) if maps else None,occupied_cells_max=max((len(r['occupied']) for r in maps),default=0),stale_status_count=sum(r['kind']=='status' and r['data']['stale'] for r in selected))
    if target and source:
        candidates=points[roi(points)]
        pm['target_roi']=dict(points=len(candidates),frames_with_return=sum(bool(np.any(roi(cloud_points(r)))) for r in source),frame_fraction=sum(bool(np.any(roi(cloud_points(r)))) for r in source)/len(source))
        if len(candidates):
            delta=candidates-center
            outside=np.maximum(np.maximum(bounds[:,0]-candidates,candidates-bounds[:,1]),0)
            pm['target_roi'].update(median_xyz_m=np.median(candidates,axis=0).tolist(),median_vector_from_deepest_center_m=np.median(delta,axis=0).tolist(),center_distance_p50_m=percentile(np.linalg.norm(delta,axis=1),50),center_distance_p95_m=percentile(np.linalg.norm(delta,axis=1),95),distance_outside_physical_box_p50_m=percentile(np.linalg.norm(outside,axis=1),50),distance_outside_physical_box_p95_m=percentile(np.linalg.norm(outside,axis=1),95))
    phases[phase]=dict(rows=selected,source=source,processed=processed,maps=maps,points=points,cells=cells,voxel=voxel)
    metrics['phase_metrics'][phase]=pm

if target:
    metrics['reflector_envelope_base_footprint_m']=bounds.tolist()
if 'baseline' in phases and 'reflector' in phases and target:
    base,present=phases['baseline'],phases['reflector']
    bn,pn=len(base['source']),len(present['source'])
    keys={k for k,v in present['voxel'].items() if v/pn>=.20 and v/pn-base['voxel'][k]/bn>=.20}
    candidates=np.array([p for p in present['points'] if tuple(np.floor(p/.1).astype(int)) in keys]).reshape((-1,3))
    association=dict(voxel_size_m=.1,required_present_frame_fraction=.20,required_fraction_increase=.20,selected_voxels=len(keys),selected_points=len(candidates),selected_voxel_centers_m=[[(v+.5)*.1 for v in k] for k in sorted(keys)])
    if len(candidates):
        median=np.median(candidates,axis=0)
        outside=np.maximum(np.maximum(bounds[:,0]-candidates,candidates-bounds[:,1]),0)
        association.update(median_xyz_m=median.tolist(),median_minus_deepest_center_m=(median-center).tolist(),median_center_residual_norm_m=float(np.linalg.norm(median-center)),distance_outside_physical_box_p50_m=percentile(np.linalg.norm(outside,axis=1),50),distance_outside_physical_box_p95_m=percentile(np.linalg.norm(outside,axis=1),95))
    cluster_metrics=[]
    for key in sorted(keys):
        entry=dict(voxel=[int(v) for v in key],phase={})
        entry['grouping']='0.1 m forward range band through the predefined 3-D ROI; includes height changes between conditions'
        for phase,data in phases.items():
            pp=np.array([p for p in data['points'][roi(data['points'])] if int(math.floor(p[0]/.1))==key[0]]).reshape((-1,3))
            if len(pp):
                entry['phase'][phase]=dict(points=len(pp),median_xyz_m=np.median(pp,axis=0).tolist(),p05_xyz_m=np.percentile(pp,5,axis=0).tolist(),p95_xyz_m=np.percentile(pp,95,axis=0).tolist())
        cluster_metrics.append(entry)
    association['clusters']=cluster_metrics
    association['interpretation']='Assembly-associated candidate modes; pooled median can lie between distinct modes and is NOT point-target range accuracy.'
    metrics['baseline_contrast_association']=association
    bmapn,pmapn=len(base['maps']),len(present['maps'])
    exclusive=[]
    for i,v in present['cells'].items():
        x=-10+(i%200+.5)*.1;y=-10+(i//200+.5)*.1
        if bounds[0,0]-.1<=x<=bounds[0,1]+.1 and bounds[1,0]-.1<=y<=bounds[1,1]+.1 and v/pmapn>=.50 and base['cells'][i]/bmapn<=.05:exclusive.append(i)
    metrics['target_exclusive_map_cells']=dict(indices=sorted(exclusive),selection='Within physical xy footprint plus 0.1 m margin; occupied >=50% present maps and <=5% baseline maps.')
    if 'removal_ack' in times:
        ack=times['removal_ack']['monotonic'];request=times['removal_request']['monotonic']
        after=[r for r in rows if r['kind']=='occupancy' and ack<=r['monotonic']<=times.get('post_removal_end',{'monotonic':rows[-1]['monotonic']})['monotonic']]
        # Examine each cell, and retain recurrences rather than claiming permanent removal.
        cell_results=[]
        for i in sorted(exclusive):
            rr=[(r,dict(r['occupied']).get(i)) for r in after]
            first=next((r for r,v in rr if v is None),None)
            occupied_after=[r for r,v in rr if v is not None]
            # Actual unknown must be distinguished from merely non-occupied known values.
            # This pipeline has endpoint-only positive evidence, so known==occupied here.
            cell_results.append(dict(index=i,first_not_occupied_after_ack_s=first['monotonic']-ack if first else None,occupied_fraction_after_ack=len(occupied_after)/len(rr) if rr else None,last_occupied_after_ack_s=occupied_after[-1]['monotonic']-ack if occupied_after else None))
        metrics['removal']=dict(request_unix=times['removal_request']['unix'],ack_unix=times['removal_ack']['unix'],action_time_uncertainty_s=ack-request,cells=cell_results,all_selected_cells_not_occupied_after_ack_s=next((r['monotonic']-ack for r in after if exclusive and not any(i in dict(r['occupied']) for i in exclusive)),None),note='Receipt-timed physical removal with acquisition continuing; no serial stop used. First disappearance is not proof of permanent absence; recurrence fractions retained.')
        # Keep the foil/marker-only outcome separate from later removal of boxes.
        phase_after=phases.get('post_removal',{}).get('maps',[])
        metrics['removal']['boxes_only_cell_occupied_fractions']={str(i):sum(i in dict(r['occupied']) for r in phase_after)/len(phase_after) for i in exclusive} if phase_after else {}
    if 'clear_ack' in times:
        ack=times['clear_ack']['monotonic'];request=times['clear_request']['monotonic']
        after=[r for r in rows if r['kind']=='occupancy' and r['monotonic']>=ack and r['monotonic']<=times.get('cleared_end',{'monotonic':rows[-1]['monotonic']})['monotonic']]
        cell_results=[]
        for i in sorted(exclusive):
            before=[r for r in rows if r['kind']=='occupancy' and r['monotonic']<=ack and i in dict(r['occupied'])]
            post=[r for r in after if i in dict(r['occupied'])]
            first=next((r for r in after if i not in dict(r['occupied'])),None)
            last_known=(post[-1] if post else before[-1] if before else None)
            cell_results.append(dict(index=i,first_not_occupied_after_ack_s=first['monotonic']-ack if first else None,occupied_fraction_after_ack=len(post)/len(after) if after else None,last_occupied_relative_to_ack_s=last_known['monotonic']-ack if last_known else None))
        metrics['final_clear']=dict(request_unix=times['clear_request']['unix'],ack_unix=times['clear_ack']['unix'],action_time_uncertainty_s=ack-request,cells=cell_results,all_selected_cells_not_occupied_after_ack_s=next((r['monotonic']-ack for r in after if exclusive and not any(i in dict(r['occupied']) for i in exclusive)),None),note='All added objects absent; acquisition continues. Absence of occupied evidence becomes unknown, not demonstrated free space.')

colors={'baseline':'#517b9f','reflector':'#d16a28','post_removal':'#3e9675','cleared':'#825d9c'}
labels={'baseline':'empty baseline','reflector':'foil + marker + boxes','post_removal':'boxes only','cleared':'all added objects removed'}
fig,axes=plt.subplots(2,max(1,len(phases)),figsize=(5*max(1,len(phases)),8),squeeze=False)
for col,(name,data) in enumerate(phases.items()):
    points=data['points']
    for row,idx,label in [(0,1,'left y (m)'),(1,2,'height z (m)')]:
        ax=axes[row,col]
        if len(points):ax.scatter(points[:,0],points[:,idx],s=3,alpha=.15,color=colors[name],rasterized=True)
        if target:
            ax.add_patch(Rectangle((bounds[0,0],bounds[idx,0]),np.diff(bounds[0])[0],np.diff(bounds[idx])[0],fill=False,color='black',linewidth=1.5,label='physical envelope'))
            ax.plot(center[0],center[idx],'+k',markersize=10)
            ax.set_xlim(max(-.1,bounds[0,0]-.8),bounds[0,1]+.8)
            ax.set_ylim(bounds[idx,0]-.6,bounds[idx,1]+.6)
        else:ax.set_xlim(-.2,3);ax.set_ylim(-1.5,1.5)
        ax.set(xlabel='forward x (m)',ylabel=label,title=labels[name]);ax.grid(alpha=.25)
fig.suptitle('Real source detections transformed by surveyed mount; all returns retained')
fig.tight_layout();fig.savefig(out/'spatial_returns.png',dpi=170);fig.savefig(out/'spatial_returns.pdf');plt.close(fig)

fig,axes=plt.subplots(1,max(1,len(phases)),figsize=(5*max(1,len(phases)),4),squeeze=False)
for ax,(name,data) in zip(axes[0],phases.items()):
    image=np.zeros((200,200));n=len(data['maps'])
    for i,count in data['cells'].items():image[i//200,i%200]=count/max(1,n)
    im=ax.imshow(image,origin='lower',extent=[-10,10,-10,10],vmin=0,vmax=1,cmap='magma',interpolation='nearest')
    if target:ax.add_patch(Rectangle((bounds[0,0],bounds[1,0]),np.diff(bounds[0])[0],np.diff(bounds[1])[0],fill=False,color='cyan',linewidth=1.5))
    ax.set(xlim=(-.2,2.5),ylim=(-1,1.2),xlabel='forward x (m)',ylabel='left y (m)',title=labels[name])
fig.colorbar(im,ax=list(axes[0]),label='fraction of maps occupied');fig.suptitle('Measured occupancy frequency (zero includes unknown)');fig.savefig(out/'occupancy_frequency.png',dpi=170,bbox_inches='tight');plt.close(fig)

if phases:
    beginning=times['baseline_begin']['monotonic']
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for kind,color in [('source','#517b9f'),('processed','#d16a28')]:
        rr=[r for r in rows if r['kind']==kind and r['monotonic']>=beginning]
        counts=[int(np.sum(roi(cloud_points(r)))) if target else len(r['points']) for r in rr]
        axes[0].plot([r['monotonic']-beginning for r in rr],counts,color=color,label=kind,alpha=.75,linewidth=.7)
    maps=[r for r in rows if r['kind']=='occupancy' and r['monotonic']>=beginning]
    selected=set(metrics.get('target_exclusive_map_cells',{}).get('indices',[]))
    axes[1].plot([r['monotonic']-beginning for r in maps],[len(selected.intersection(i for i,v in r['occupied'])) if target else len(r['occupied']) for r in maps],color='#3e9675')
    status=[r for r in rows if r['kind']=='status' and r['monotonic']>=beginning]
    axes[2].step([r['monotonic']-beginning for r in status],[int(r['data']['stale']) for r in status],where='post')
    for name in ['baseline_begin','baseline_end','reflector_begin','reflector_end','removal_request','removal_ack','post_removal_end','clear_request','clear_ack','cleared_end']:
        if name in times:
            x=times[name]['monotonic']-beginning
            for ax in axes:ax.axvline(x,color='gray',alpha=.4,linestyle='--')
            axes[0].text(x,1.02,name,rotation=65,transform=axes[0].get_xaxis_transform(),fontsize=7)
    axes[0].set_ylabel('ROI returns/frame' if target else 'returns/frame');axes[0].legend()
    axes[1].set_ylabel('target-exclusive cells' if target else 'occupied cells')
    axes[2].set(ylabel='stale flag',xlabel='seconds since fresh baseline began',ylim=(-.1,1.1))
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'removal_timeline.png',dpi=170);plt.close(fig)

if target and phases:
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for name,data in phases.items():
        pp=data['points'][roi(data['points'])]
        if len(pp):
            axes[0].hist(pp[:,0],bins=np.arange(bounds[0,0]-.12,bounds[0,1]+.121,.01),weights=np.ones(len(pp))/max(1,len(data['source'])),histtype='step',color=colors[name],label=labels[name])
            axes[1].hist(pp[:,2],bins=np.arange(bounds[2,0]-.2,bounds[2,1]+.201,.01),weights=np.ones(len(pp))/max(1,len(data['source'])),histtype='step',color=colors[name],label=labels[name])
    for ax,idx in zip(axes,[0,2]):
        ax.axvspan(bounds[idx,0],bounds[idx,1],color='gray',alpha=.15,label='foil physical extent')
        ax.axvline(center[idx],color='black',linestyle='--',label='recessed center')
        ax.set(xlabel=('forward x (m)' if idx==0 else 'height z (m)'),ylabel='returns per source frame per bin');ax.grid(alpha=.2)
    axes[0].legend(fontsize=8);fig.suptitle('ROI return distributions: empty baseline and all-removed phases had zero ROI returns')
    fig.tight_layout();fig.savefig(out/'range_height_distributions.png',dpi=170);plt.close(fig)

(out/'spatial_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps(metrics,indent=2))
