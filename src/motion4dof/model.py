"""Centimetres/degrees; Z base and three parallel pitch joints. Not a 5R arm."""
import numpy as np


class MotionError(ValueError):
    pass


def vector(value, n):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise MotionError(f'Expected {n} finite values')
    return a


class Arm4:
    def __init__(self, lengths=(8, 24, 24, 8), limits=((-135,135),(-90,90),(0,150),(-90,40))):
        self.lengths = vector(lengths, 4)
        self.limits = np.asarray(limits, dtype=float)
        if (np.any(self.lengths <= 0) or self.limits.shape != (4,2)
                or not np.isfinite(self.limits).all() or np.any(self.limits[:,0] >= self.limits[:,1])):
            raise MotionError('Invalid geometry or limits')

    def check(self, q):
        q = vector(q, 4)
        if np.any(q < self.limits[:,0]-1e-8) or np.any(q > self.limits[:,1]+1e-8):
            raise MotionError('Joint limit exceeded')
        return q

    def points(self, q):
        base, shoulder, elbow, wrist = np.radians(self.check(q))
        h, a, b, tool = self.lengths
        positions = [np.zeros(3), np.array([0.,0.,h])]
        for length, pitch in ((a,shoulder),(b,shoulder-elbow),(tool,shoulder-elbow+wrist)):
            positions.append(positions[-1] + length*np.array([
                np.cos(pitch)*np.cos(base), np.cos(pitch)*np.sin(base), np.sin(pitch)]))
        return np.array(positions)

    def forward(self, q):
        q = self.check(q)
        return self.points(q)[-1], float(q[1]-q[2]+q[3])

    def inverse(self, target, pitch_deg, seed=(0,60,30,0)):
        p, seed = vector(target,3), self.check(seed)
        if not np.isfinite(pitch_deg):
            raise MotionError('Invalid pitch')
        h,a,b,tool = self.lengths
        pitch = np.radians(pitch_deg)
        radius = np.hypot(*p[:2])
        heading = np.arctan2(p[1],p[0]) if radius > 1e-9 else np.radians(seed[0])
        solutions=[]
        # A folded arm can place the TCP behind the base heading.
        for base,r in ((heading,radius),(heading+np.pi,-radius)):
            base = (base+np.pi)%(2*np.pi)-np.pi
            rw,zw = r-tool*np.cos(pitch), p[2]-h-tool*np.sin(pitch)
            c = (rw*rw+zw*zw-a*a-b*b)/(2*a*b)
            if abs(c)>1+1e-9:
                continue
            for elbow in (np.arccos(np.clip(c,-1,1)), -np.arccos(np.clip(c,-1,1))):
                shoulder = np.arctan2(zw,rw)+np.arctan2(b*np.sin(elbow),a+b*np.cos(elbow))
                wrist = pitch-shoulder+elbow
                q=np.degrees([base,shoulder,elbow,wrist])
                try:
                    self.check(q)
                except MotionError:
                    continue
                if np.linalg.norm(self.forward(q)[0]-p)<1e-6:
                    solutions.append(q)
        if not solutions:
            raise MotionError('Target position/pitch unreachable within joint limits')
        return min(solutions,key=lambda q: np.linalg.norm(q-seed))

    def face(self, target, shoulder=60., elbow=30.):
        p=vector(target,3)
        base=np.degrees(np.arctan2(p[1],p[0]))
        wrist_pos=self.points([base,shoulder,elbow,0])[-2]
        delta=p-wrist_pos
        radial=delta[0]*np.cos(np.radians(base))+delta[1]*np.sin(np.radians(base))
        if np.linalg.norm(delta)<=self.lengths[3]:
            raise MotionError('Face target too close to tool')
        pitch=np.degrees(np.arctan2(delta[2],radial))
        return self.check([base,shoulder,elbow,pitch-shoulder+elbow])

    def book(self, target, distance=30., seed=(0,60,30,0)):
        p=vector(target,3)
        if not np.isfinite(distance) or distance<=0:
            raise MotionError('Invalid working distance')
        r=np.hypot(*p[:2])
        direction=p[:2]/r if r>1e-9 else np.array([1.,0.])
        offset=np.r_[-min(.25*r,15)*direction,30.]
        tcp=p+offset/np.linalg.norm(offset)*distance
        delta=p-tcp
        pitch=np.degrees(np.arctan2(delta[2],np.dot(delta[:2],direction)))
        return self.inverse(tcp,pitch,seed)


def trajectory(arm,start,end,duration=3.,steps=30,max_velocity=60.):
    start,end=arm.check(start),arm.check(end)
    if not np.isfinite([duration,max_velocity]).all() or duration<=0 or max_velocity<=0:
        raise MotionError('Invalid timing/velocity')
    if isinstance(steps,bool) or not isinstance(steps,int) or steps<1:
        raise MotionError('steps must be a positive integer')
    delta=end-start
    duration=max(float(duration),float(1.875*np.max(np.abs(delta))/max_velocity))
    u=np.linspace(0,1,steps+1)
    blend=10*u**3-15*u**4+6*u**5
    return u*duration,start+blend[:,None]*delta
