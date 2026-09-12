"""Alldro always has FOUR positioning angles; GRIP controls channel 4 alone."""
import numpy as np
from .model import Arm4, MotionError


def arm_command(q):
    angles=Arm4().check(q)
    rounded=np.rint(angles).astype(int)
    Arm4().check(rounded)
    return 'Alldro '+' '.join(map(str,rounded))+'\n'


def gripper_command(angle):
    if isinstance(angle,bool) or not np.isscalar(angle) or not np.isfinite(angle) or not 0<=angle<=180:
        raise MotionError('Gripper angle must be 0..180 degrees')
    return f'GRIP {int(round(angle))}\n'


class SimReceiver:
    def __init__(self):
        self.joints=[0.,60.,30.,0.]
        self.gripper=None
        self.frames=[]

    def accept(self,line):
        parts=line.split()
        if not parts:
            raise MotionError('Empty frame')
        if parts[0]=='Alldro' and len(parts)==5:
            q=Arm4().check([float(v) for v in parts[1:]])
            self.joints=q.tolist()
        elif parts[0]=='GRIP' and len(parts)==2:
            angle=float(parts[1]); gripper_command(angle)
            self.gripper=angle
        else:
            raise MotionError('Expected four arm angles or one gripper angle')
        self.frames.append(line.strip())
