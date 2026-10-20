"""Local U-turn behaviour cloning with NumPy/SciPy; no downloaded checkpoint or robot deployment.

python -m tests.policy.train_uturn_specialist --output build/uturn-specialist
Episode-level split: training radii/seeds differ from closed-loop holdout evaluation.
Original AngularUTurn stays unchanged and is reported independently, never included as a success.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from policy.action_policy.mpc import MpcConfig
from policy.action_policy.policy import MpcPolicy
from policy.action_policy.uturn_specialist import UTurnStudentController,features
from policy.cone_filter.cone_filter import ConeFilterConfig,filter_cones
from policy.lane_detection.lane_detection import LaneDetectionConfig,LaneDetector
from policy.lane_detection.temporal import detect_temporal_lane
from policy.memory import CarMemory
from tests.policy.angular_lanes import AngularUTurn
from tests.policy.curved_lanes import CurveCase,CurvedLane,run_curve


def pipeline(controller):
    memory=CarMemory();detector=LaneDetector(LaneDetectionConfig(max_curvature_per_m=2,single_edge_hold_enabled=True))
    def observe(obs):
        obs=replace(obs,cones=filter_cones(obs.cones,ConeFilterConfig(check_straight_rows=False)))
        lane,_=detect_temporal_lane(obs,memory,detector)
        memory.step()
        return lane
    original=controller.run_policy
    def run(obs,lane):
        return original(obs,lane,lane_support_m=lane.visible_until_m if memory.data.lane_tracking.holding_single_edge else None)
    controller.run_policy=run
    return observe


def curved(radius,sign,seed,occlusion=False,start=0):
    return CurveCase(CurvedLane(pieces=(('straight',1.),('arc',sign/radius,180.),('straight',1.))),
                     seed=seed,start_left_m=start,max_duration_s=45,
                     inner_occlusion_s=(8.,8.4) if occlusion else None)


def train_network(x,y,path,config):
    mean=x.mean(0);scale=np.maximum(x.std(0),.05);z=(x-mean)/scale;target=y/.92
    shapes=((7,32),(32,),(32,32),(32,),(32,y.shape[1]),(y.shape[1],))
    sizes=[int(np.prod(shape)) for shape in shapes];ends=np.cumsum([0]+sizes)
    def unpack(vector):return [vector[ends[i]:ends[i+1]].reshape(shape) for i,shape in enumerate(shapes)]
    rng=np.random.default_rng(4)
    initial=np.concatenate([(rng.normal(0,.12,shape) if len(shape)==2 else np.zeros(shape)).ravel() for shape in shapes])
    def objective(vector):
        w1,b1,w2,b2,w3,b3=unpack(vector)
        h1=np.tanh(z@w1+b1);h2=np.tanh(h1@w2+b2);prediction=h2@w3+b3;error=prediction-target
        loss=np.mean(error**2)+1e-6*np.sum(vector**2)
        d=2*error/error.size
        gw3=h2.T@d;gb3=d.sum(0);d2=(d@w3.T)*(1-h2*h2)
        gw2=h1.T@d2;gb2=d2.sum(0);d1=(d2@w2.T)*(1-h1*h1)
        gw1=z.T@d1;gb1=d1.sum(0)
        gradient=np.concatenate([a.ravel() for a in (gw1,gb1,gw2,gb2,gw3,gb3)])+2e-6*vector
        return loss,gradient
    result=minimize(objective,initial,jac=True,method='L-BFGS-B',options={'maxiter':400,'ftol':1e-10,'maxcor':12})
    arrays=dict(zip(('w1','b1','w2','b2','w3','b3'),unpack(result.x)))
    np.savez(path,mean=mean,scale=scale,contract=np.array([config.max_curvature_per_m,config.max_right_curvature_per_m,config.step_s,config.horizon_steps]),**arrays)
    return {'samples':len(x),'iterations':int(result.nit),'loss':float(result.fun),'optimiser_success':bool(result.success),'message':str(result.message)}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=Path('build/uturn-specialist'));args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    config=MpcConfig(turn_slowdown_enabled=True,run_duration_s=0)
    inputs=[];targets=[];episodes=[]
    for index,(radius,sign,start) in enumerate((r,s,o) for r in (2.5,3.2,4.) for s in (1,-1) for o in (-.08,.08)):
        controller=MpcPolicy(config);solve=controller.controller.solve
        def collect(state,previous,age,speed_m_per_s=None,*,lateral_offset_m=0):
            result=solve(state,previous,age,speed_m_per_s,lateral_offset_m=lateral_offset_m)
            if result.converged:
                inputs.append(features(state,previous,age,speed_m_per_s,lateral_offset_m));targets.append(result.curvatures_per_m.copy())
            return result
        controller.controller.solve=collect
        case=curved(radius,sign,index,start=start)
        result=run_curve(case,controller,pipeline(controller))
        episodes.append({'radius':radius,'direction':sign,'seed':index,'start_left_m':start,'lane_kept':result.lane_kept})
        print('demonstration',index+1,'samples',len(inputs),flush=True)
    x,y=np.array(inputs),np.array(targets);np.savez(out/'demonstrations.npz',x=x,y=y)
    fit=train_network(x,y,out/'uturn-steering.npz',config);print('trained',fit,flush=True)
    report={'method':'MPC behaviour cloning, 7->32->32->12 tanh network; trained from scratch, not downloaded pretrained weights',
            'model_is_offline_only':True,'training_episodes':episodes,'training':fit,'evaluation':{}}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(2,3,figsize=(13,8))
    cases={'holdout_left_r2.8':curved(2.8,1,101), 'holdout_right_r2.8':curved(2.8,-1,102),
           'holdout_left_r3.6':curved(3.6,1,103), 'holdout_occlusion':curved(2.8,1,104,True),
           'original_three_line':CurveCase(AngularUTurn(),seed=105,max_duration_s=45),
           'original_three_line_occlusion':CurveCase(AngularUTurn(),seed=106,max_duration_s=45,inner_occlusion_s=(4,4.4))}
    for (name,case),axis in zip(cases.items(),axes.flat):
        report['evaluation'][name]={}
        for mode in ('teacher','student'):
            controller=MpcPolicy(config)
            if mode=='student':controller.controller=UTurnStudentController(config,out/'uturn-steering.npz')
            result=run_curve(case,controller,pipeline(controller))
            report['evaluation'][name][mode]={**result.metrics,'lane_kept':result.lane_kept,'inference_stop':controller.stopped_for_solver_failure}
            axis.plot(result.x,result.y,label=mode)
        for colour,_,cx,cy in case.lane.cones():axis.plot(cx,cy,'.',color='royalblue' if colour.name=='BLUE' else 'goldenrod',markersize=2)
        axis.set_title(name.replace('_',' '));axis.set_aspect('equal');axis.legend(fontsize=8)
        print(name,report['evaluation'][name],flush=True)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    figure.suptitle('U-turn-only learned steering: held-out layouts and unchanged hard benchmark');figure.tight_layout();figure.savefig(out/'evaluation.png',dpi=150)
    sources=('policy/action_policy/uturn_specialist.py',__file__,'policy/action_policy/mpc.py','policy/lane_detection/temporal.py','tests/policy/curved_lanes.py','tests/policy/angular_lanes.py')
    report['source_sha256']={str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sources}
    report['limitations']='Nominal no-slip simulation with valid gyro, forward only, unchanged .48/.92 limits; no hardware or standalone safety qualification. Success on smooth training layouts cannot establish feasibility of the original angular U.'
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
