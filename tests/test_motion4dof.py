import importlib
import shutil
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from src.motion4dof import Arm4, MotionError, trajectory, arm_command, gripper_command, SimReceiver
from src.motion4dof.__main__ import run
from src.object_tracking import ObjectTracker


class FourAxisTests(unittest.TestCase):
    def test_known_fk(self):
        arm=Arm4()
        np.testing.assert_allclose(arm.forward([0,0,0,0])[0],[56,0,8])
        np.testing.assert_allclose(arm.forward([90,0,0,0])[0],[0,56,8],atol=1e-9)

    def test_300_roundtrips(self):
        a=Arm4(); rng=np.random.default_rng(20260911)
        for _ in range(300):
            q=rng.uniform(a.limits[:,0]+.1,a.limits[:,1]-.1)
            p,pitch=a.forward(q); recovered=a.inverse(p,pitch,q)
            np.testing.assert_allclose(a.forward(recovered)[0],p,atol=1e-6)
            self.assertAlmostEqual(a.forward(recovered)[1],pitch,places=7)

    def test_invalid_and_unreachable(self):
        a=Arm4()
        for q in ([0]*5,[0,100,0,0],[float('nan')]*4):
            with self.assertRaises(ValueError): a.forward(q)
        with self.assertRaises(MotionError): a.inverse([1000,0,0],0)

    def test_face_aim_and_book(self):
        a=Arm4()
        for target,q in ((np.array([55,0,40]),a.face([55,0,40])),(np.array([40,5,0]),a.book([40,5,0]))):
            tcp,pitch=a.forward(q)
            direction=np.array([np.cos(np.radians(pitch))*np.cos(np.radians(q[0])),
                                np.cos(np.radians(pitch))*np.sin(np.radians(q[0])),np.sin(np.radians(pitch))])
            np.testing.assert_allclose((target-tcp)/np.linalg.norm(target-tcp),direction,atol=1e-8)
        self.assertAlmostEqual(np.linalg.norm(a.forward(a.book([40,5,0]))[0]-[40,5,0]),30.)

    def test_tracking_adapter_uses_actual_fk(self):
        t=ObjectTracker(); q=t.track_book([40,5,0])
        actual=t.book_forward_kinematics(*q,end_effector_pos=[999,999,999])[-1]
        np.testing.assert_allclose(actual,t.arm.forward(np.degrees(q))[0])
        base,wrist=t.track_face([55,0,40]); self.assertLess(wrist,0)
        np.testing.assert_allclose(t.face_forward_kinematics(base,wrist)[-1],t.calculate_face_ik([55,0,40])[2])

    def test_trajectory_limits_and_speed(self):
        a=Arm4(); start=[0,60,30,0]; end=a.book([40,5,0])
        times,q=trajectory(a,start,end,.01,100,max_velocity=20)
        np.testing.assert_allclose(q[0],start); np.testing.assert_allclose(q[-1],end)
        self.assertLessEqual(np.max(np.abs(np.diff(q,axis=0)/np.diff(times)[:,None])),20.001)
        for row in q: a.check(row)
        for steps in (0,True,1.2):
            with self.assertRaises(ValueError): trajectory(a,start,end,steps=steps)

    def test_independent_gripper_and_arity(self):
        r=SimReceiver(); r.accept(arm_command([10,20,30,0])); before=r.joints.copy()
        r.accept(gripper_command(80)); self.assertEqual(r.joints,before); self.assertEqual(r.gripper,80)
        r.accept(arm_command([0,20,30,0])); self.assertEqual(r.gripper,80)
        for line in ('Alldro 1 2 3\n','Alldro 1 2 3 4 5\n','GRIP nan\n','GRIP 181\n','GRIP 10 20\n'):
            with self.assertRaises(ValueError): r.accept(line)

    def test_demo_and_faults(self):
        normal=run(); self.assertEqual(normal['status'],'DONE'); self.assertEqual(normal['arm_dof'],4)
        self.assertEqual([f for f in normal['frames'] if f.startswith('GRIP')],['GRIP 20','GRIP 80','GRIP 20'])
        self.assertEqual(run('unreachable')['frames'],[])
        for fault,count in [('transport',5),('estop',8)]:
            r=run(fault); self.assertEqual(r['status'],'HALTED'); self.assertEqual(len(r['samples']),count)
            self.assertEqual(len(r['frames']),count+1)

    def test_serial_writer_failure_and_gripper(self):
        module=importlib.import_module('src.serial_comm')
        comm=module.SerialCommunicator.__new__(module.SerialCommunicator)
        class Port:
            is_open=True
            def __init__(self): self.lines=[]; self.fail=False
            def write(self,data):
                if self.fail: return 0
                self.lines.append(data); return len(data)
            def close(self): self.is_open=False
        comm.ser=Port(); comm.running=False; comm.faulted=False
        comm.tx_lock=threading.Lock(); comm.motion_lock=threading.Lock()
        comm.current_angles=[0,60,30,0]; comm.gripper_angle=None
        comm.send_gripper(80); self.assertEqual(comm.current_angles,[0,60,30,0])
        with patch.object(module.time,'sleep'):
            comm.send_servos_smooth([0,60,30,-20],.1,2)
        self.assertEqual(comm.gripper_angle,80); self.assertEqual(comm.current_angles,[0,60,30,-20])
        before=comm.current_angles.copy(); comm.ser.fail=True
        with patch.object(module.time,'sleep'), self.assertRaises(RuntimeError):
            comm.send_servos_smooth([0,60,30,0],.1,2)
        self.assertTrue(comm.faulted); self.assertEqual(comm.current_angles,before)
        with self.assertRaises(RuntimeError): comm.send_gripper(20)

    def test_actual_firmware_motion_handler(self):
        compiler=shutil.which('gcc')
        if not compiler: self.skipTest('Host GCC required')
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            exe=Path(temp)/'motion.exe'
            subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror','-I',str(root/'firmware/Core/Inc'),str(root/'tests/lamp_motion_host.c'),'-o',str(exe)],check=True,capture_output=True)
            frames=run()['frames']
            invalid=['Alldro 1 2 3','Alldro 1 2 3 4 5','Alldro 1 2 3 bad','GRIP nan','GRIP 181','GRIP 3 4','GRIP','Alldro 200 2 3 4']
            output=subprocess.run([str(exe)],input='\n'.join(frames+invalid)+'\n',text=True,capture_output=True,check=True).stdout.splitlines()
            self.assertEqual(len(output),len(frames)+len(invalid))
            for frame,result in zip(frames,output):
                p=frame.split(); channels=[4] if p[0]=='GRIP' else range(4)
                expected=''.join(f'{ch}:{float(value):.3f} ' for ch,value in zip(channels,p[1:]))+'result=1'
                self.assertEqual(result,expected)
            self.assertTrue(all(row=='result=-1' for row in output[len(frames):]))


if __name__=='__main__': unittest.main()
