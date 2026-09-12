"""Offline four-joint + independent gripper command demonstration."""
import argparse
import json
from pathlib import Path
import numpy as np
from . import Arm4, trajectory, arm_command, gripper_command, SimReceiver


def run(fault=None):
    arm=Arm4(); receiver=SimReceiver(); samples=[]
    report={'input_source':'SYNTHETIC targets; example geometry; no camera/UART/PWM',
            'arm_dof':4,'actuator_channels':5,'gripper_independent':True,
            'lengths_cm':arm.lengths.tolist(),'limits_deg':arm.limits.tolist(),
            'face_target_cm':[55,0,40],'book_target_cm':[40,5,0],
            'status':'DONE','samples':samples,'frames':receiver.frames}
    q=np.array([0.,60.,30.,0.]); clock=0.
    try:
        if fault=='unreachable':
            arm.inverse([1000,0,0],0)
        plan=[('face',arm.face(report['face_target_cm'])),
              ('book',arm.book(report['book_target_cm'])),('home',q)]
        # Preflight all IK, trajectory and encodings before first command.
        segments=[]
        previous=q
        for name,end in plan:
            times,joints=trajectory(arm,previous,end,steps=40)
            segments.append((name,times,joints,[arm_command(v) for v in joints]))
            previous=end
        receiver.accept(gripper_command(20))
        for name,times,joints,commands in segments:
            for t,planned,line in zip(times,joints,commands):
                if fault=='transport' and len(samples)==5:
                    raise RuntimeError('SIMULATED_UART_FAILURE: no further commands')
                if fault=='estop' and len(samples)==8:
                    raise RuntimeError('SIMULATED_STOP: no further commands')
                receiver.accept(line)
                actual,pitch=arm.forward(receiver.joints)
                samples.append({'t':clock+float(t),'phase':name,'joints_deg':list(receiver.joints),
                                'planned_deg':planned.tolist(),'gripper_deg':receiver.gripper,
                                'tcp_cm':actual.tolist(),'points_cm':arm.points(receiver.joints).tolist(),
                                'pitch_deg':pitch,'quantization_mm':float(np.linalg.norm(actual-arm.forward(planned)[0])*10)})
            clock+=float(times[-1])
            if name=='book':
                # Exercise separate channel; this does not assert a successful grasp.
                for angle in (80,20):
                    receiver.accept(gripper_command(angle)); clock+=.5
                    row=dict(samples[-1]); row.update(t=clock,phase='gripper',gripper_deg=angle)
                    samples.append(row)
    except (ValueError,RuntimeError) as exc:
        report.update(status='HALTED',reason=str(exc))
    report['virtual_duration_s']=samples[-1]['t'] if samples else 0.
    report['max_quantization_mm']=max((s['quantization_mm'] for s in samples),default=0.)
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='output/four-axis-demo.json')
    parser.add_argument('--fault',choices=['unreachable','transport','estop'])
    args=parser.parse_args(); report=run(args.fault)
    path=Path(args.output); path.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps(report,ensure_ascii=False,indent=2)
    path.write_text(payload,encoding='utf-8')
    html=Path(__file__).with_name('report.html').read_text(encoding='utf-8').replace('__DATA__',payload)
    path.with_suffix('.html').write_text(html,encoding='utf-8')
    print(json.dumps({'status':report['status'],'samples':len(report['samples']),
                      'frames':len(report['frames']),'report':str(path)},ensure_ascii=False))
    return 0 if report['status']=='DONE' else 2


if __name__=='__main__':
    raise SystemExit(main())
