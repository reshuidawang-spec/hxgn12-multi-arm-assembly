"""Replay a completed R1..Rn prefix, checking real held parts and release poses."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS, PICK_PART, PLACE_PART, ROBOT_IDS, SCENE_FILE,
    PLAN_SCHEMA_VERSION, Scene, AssemblyRuntime, RemoteAPIClient,
    fingerprint, motion_policy_matches, planning_product_state,
)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--through',choices=ROBOT_IDS,required=True)
    p.add_argument('--plan',type=Path,default=Path('data/fixed_paths/eight_arm_cabinet.partial.json'))
    p.add_argument('--port',type=int,default=23000)
    p.add_argument('--adopt-scene',action='store_true')
    a=p.parse_args()
    plan=json.loads(a.plan.read_text())
    if plan['schema_version'] != PLAN_SCHEMA_VERSION or not motion_policy_matches(plan):
        raise RuntimeError('planner schema or policy is stale')
    if not a.adopt_scene and plan['scene']['sha256'] != fingerprint(SCENE_FILE)['sha256']:
        raise RuntimeError('scene changed; audit explicitly with --adopt-scene')
    process_order=['R1','R2','R3','R4','R5','R6','R8','R7']
    robots=process_order[:process_order.index(a.through)+1]
    for robot in robots:
        if set(ACTION_TARGETS[robot])-set(plan['actions'].get(robot,{})):
            raise RuntimeError(f'{robot} paths are incomplete')
    client=RemoteAPIClient('127.0.0.1',a.port)
    client.timeout=900
    scene=Scene(client)
    if scene.sim.getSimulationState()!=scene.sim.simulation_stopped:
        raise RuntimeError('stop simulation before preflight')
    runtime=AssemblyRuntime(scene,plan,1.)
    scene.install_batch_script()
    completed=[]
    try:
        runtime.reset_product()
        runtime.move_to_stows(False)
        runtime.simulate=False
        for robot in robots:
            for stem in ACTION_TARGETS[robot]:
                station,_=planning_product_state(robot,stem)
                if runtime.pallet_station!=station:
                    runtime.index_pallet(station,wait_for=(),emits=f'{station}_READY')
                if (robot,stem) in PICK_PART:
                    entry=runtime.pick(robot,stem,PICK_PART[(robot,stem)])
                elif (robot,stem) in PLACE_PART:
                    entry=runtime.place(robot,stem,PLACE_PART[(robot,stem)],station)
                elif robot=='R7':
                    runtime.execute_screw(runtime.track(robot,stem),int(stem[-1]),wait_for=(),emits=stem+'_DONE')
                    completed.append(f'{robot}_{stem}')
                    continue
                else:
                    entry=(runtime.track(robot,stem),lambda:None)
                runtime.execute_pair([entry],f'{robot}_{stem}')
                completed.append(f'{robot}_{stem}')
        if a.through=='R7':
            runtime.index_pallet('output',wait_for=(),emits='OUTPUT_READY')
    finally:
        runtime.reset_product()
        scene.set_all_home()
        scene.remove_batch_script()
        scene.remove_planner_script()
    if a.adopt_scene:
        plan['scene']=fingerprint(SCENE_FILE)
        # Never retain unvalidated suffix actions after a geometry change.
        plan['actions']={r:plan['actions'][r] for r in robots}
        a.plan.write_text(json.dumps(plan,indent=2)+'\n')
    report={'through':a.through,'passed':True,'completed':completed,'scene':fingerprint(SCENE_FILE)}
    out=Path('data/audits/20260908_geometry')/f'prefix_{a.through}.json'
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(f'[passed] R1..{a.through}: {len(completed)} actions, real shell and release-pose checks',flush=True)

if __name__=='__main__':
    main()
