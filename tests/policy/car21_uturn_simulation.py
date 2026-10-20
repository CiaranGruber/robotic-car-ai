"""Provisional car21 squared-U evaluation using the owner's approximate width/radius.

python -m tests.policy.car21_uturn_simulation --output build/car21-provisional-uturn
No changes to LAB_CAR calibration, runtime selection or existing track defaults. Leg/cross length remains assumed3m.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

from policy.action_policy.corridor_experiment import OfflineCorridorPolicy
from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.lane_detection.corridor import CorridorConfig
from policy.memory import CarMemory
from tests.policy.angular_lanes import AngularUTurn
from tests.policy.curved_lanes import CurveCase,run_curve
from tests.policy.real_car_measurements import LAB_CAR
from tests.policy.train_uturn_specialist import pipeline


def controller(radius=1.,speed=.4):
    return OfflineCorridorPolicy(1/radius,.92,CarMemory(),
           CorridorConfig(nominal_width_m=1.,lookahead_m=1.2,route_memory_enabled=True,route_max_age_s=6.5,route_max_distance_m=2.6),speed)


def case(radius=1.,width=1.,seed=1,speed=.4,occlusion=False):
    return CurveCase(AngularUTurn(width_m=width),seed=seed,speed_m_per_s=speed,max_duration_s=50,
                     car=replace(LAB_CAR,camera_yaw_deg=0.,max_left_curvature_per_m=1/radius),
                     inner_occlusion_s=(6.,6.4) if occlusion else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=Path('build/car21-provisional-uturn'));parser.add_argument('--seed-start',type=int,default=201);args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    report={'assumptions':{'owner_approximate_width_m':1.,'owner_approximate_left_radius_m':1.,'right_curvature_per_m':.92,'right_status':'retained car20 value, not measured car21','leg_and_cross_length_m':3.,'dimensions_status':'3m remains an assumption; only width and radius were supplied','control':'forward only, target.4m/s, lookahead1.2m; route hold<=6.5s/2.6m, no renewal on singleedge','tuning_seeds':list(range(5)),'development_seeds':list(range(101,111)),'validation_noise_seeds':list(range(args.seed_start,args.seed_start+10))},'validation':{},'sensitivity':{}}
    trajectories={}
    for seed in range(args.seed_start,args.seed_start+10):
        for occluded in (False,True):
            label=f'seed{seed}'+('_occluded' if occluded else '')
            run=run_curve(case(seed=seed,occlusion=occluded),controller())
            report['validation'][label]={**run.metrics,'lane_kept':run.lane_kept}
            if seed==args.seed_start:trajectories[label]=run
            print(label,run.lane_kept,run.metrics,flush=True)
            (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for radius in (.9,1.,1.1):
        for width in (.9,1.,1.1):
            run=run_curve(case(radius=radius,width=width,seed=21),controller(radius=radius))
            report['sensitivity'][f'radius{radius}_width{width}']={**run.metrics,'lane_kept':run.lane_kept}
    # Keep the same provisional physics/geometry for a fair comparison to the current arc-based pipeline.
    mpc=MpcPolicy(MpcConfig(target_speed_m_per_s=.4,turn_slowdown_enabled=True,max_curvature_per_m=1.,max_right_curvature_per_m=.92))
    baseline=run_curve(case(seed=args.seed_start),mpc,pipeline(mpc));trajectories['arc_baseline']=baseline
    report['arc_baseline']={**baseline.metrics,'lane_kept':baseline.lane_kept}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    lane=AngularUTurn(width_m=1.)
    for axis,occluded in zip(axes,(False,True)):
        polygon=lane._corridor_polygon();axis.fill([p[0] for p in polygon],[p[1] for p in polygon],color='lightgrey',alpha=.3)
        for colour,_,x,y in lane.cones():axis.plot(x,y,'.',color='royalblue' if colour.name=='BLUE' else 'goldenrod')
        label=f'seed{args.seed_start}'+('_occluded' if occluded else '')
        r=trajectories[label];axis.plot(r.x,r.y,label='connected path + memory',color='seagreen')
        if not occluded:axis.plot(baseline.x,baseline.y,label='arc baseline',color='indianred')
        axis.set_title('0.4s inner-cone occlusion' if occluded else 'Fresh-cone run');axis.set_aspect('equal');axis.legend();axis.set_xlabel('world x(m)');axis.set_ylabel('world y(m)')
    fig.suptitle('Provisional car21: 1m lane width, 1m left turning radius, same three-segment U');fig.tight_layout();fig.savefig(out/'validation.png',dpi=150)
    report['passed']=sum(r['lane_kept'] for r in report['validation'].values());report['total']=len(report['validation'])
    report['source_sha256']={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (__file__,'policy/action_policy/corridor_experiment.py','policy/lane_detection/corridor.py','policy/memory.py','tests/policy/curved_lanes.py','tests/policy/angular_lanes.py')}
    report['limitations']='Offline cone-only follower, not ROS-wired or vehicle-qualified. Valid simulated gyro and unsigned forward wheel speed; approximate owner geometry plus inherited camera/lag/body assumptions. No physical runtime calibration changed.'
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Passed',report['passed'],'/',report['total'],flush=True)

if __name__=='__main__':main()
