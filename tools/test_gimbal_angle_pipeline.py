import math, random
TWO_PI=2*math.pi
PI=math.pi

def wrap(x,a,b):
    r=b-a
    return (x-a)%r+a

class Tracker:
    def __init__(self, ratio=1.0, mode='continuous'):
        self.ratio=ratio
        self.mode=mode
        self.init=False
        self.last=0.0
        self.acc=0.0
        self.offset=0.0
        self.out=0.0
    def feed(self, theta):
        first=not self.init
        if first:
            self.init=True
            self.last=theta
            self.acc=theta if self.mode in ('absolute','singleturn') else 0.0
        else:
            d=theta-self.last
            if d>PI: d-=TWO_PI
            elif d<-PI: d+=TWO_PI
            self.acc+=d
            self.last=theta
        c=self.offset+self.acc/self.ratio
        self.out=wrap(c,0,TWO_PI) if self.mode=='absolute' else c
        return self.out
    def reset(self, theta, target):
        self.init=True
        self.last=theta
        self.acc=0.0
        self.offset=target
        self.out=wrap(target,0,TWO_PI) if self.mode=='absolute' else target

def nearest(config,ref):
    # Python round is bankers; emulate C roundf for our non-half cases.
    q=(ref-config)/TWO_PI
    n=math.floor(q+0.5) if q>=0 else math.ceil(q-0.5)
    return config+n*TWO_PI

random.seed(20260712)
max_err=0.0
cases=0
for ratio in (1.0,3.0,6.0):
    for direction in (-1,1):
        for _ in range(1500):
            # Start at arbitrary encoder phase, then re-zero at an arbitrary
            # finite-axis coordinate and traverse up to 320 motor degrees.
            rotor=random.uniform(0,TWO_PI)
            t=Tracker(ratio,'continuous')
            t.feed(wrap(rotor,0,TWO_PI))
            zero=random.uniform(-1.5,1.5)
            t.reset(wrap(rotor,0,TWO_PI),zero)
            true_motor=0.0
            steps=random.randint(20,300)
            total=direction*random.uniform(0,math.radians(320))
            for k in range(1,steps+1):
                next_true=total*k/steps
                theta=wrap(rotor+next_true,0,TWO_PI)
                out=t.feed(theta)
                expected=zero+next_true/ratio
                err=abs(out-expected)
                max_err=max(max_err,err)
                if err>1e-9:
                    raise SystemExit(f'tracker mismatch ratio={ratio}: {err}')
                true_motor=next_true
                cases+=1

# Explicit Atlas 6:1 case: lower-stop zero and 270 motor degrees -> 45 output degrees.
rotor=math.radians(350)
t=Tracker(6.0,'continuous')
t.feed(wrap(rotor,0,TWO_PI))
t.reset(wrap(rotor,0,TWO_PI),0.0)
for motor_deg in range(0,271):
    out=t.feed(wrap(rotor+math.radians(motor_deg),0,TWO_PI))
assert abs(out-math.radians(motor_deg)/6.0)<1e-10
assert abs(out-math.radians(45.0))<1e-10

# Minnie-style offset stored as -0.5 while the raw encoder branch is around 5.8.
center=nearest(-0.5,5.82)
assert abs(center-(TWO_PI-0.5))<1e-12
lo,hi=center-0.6,center+0.6
assert lo < 5.82 < hi

# Demonstrate the old zero-to-current shortest-delta failure at 181 rotor degrees.
old=wrap(math.radians(181),-PI,PI)/6.0
new=math.radians(181)/6.0
assert old < 0 < new
jump_deg=math.degrees(new-old)

print(f'PASS trajectories={cases}')
print(f'PASS max_tracker_error={max_err:.3e} rad')
print('PASS ratios=1,3,6')
print('PASS Atlas rotor=270deg -> output=45deg across encoder zero')
print(f'PASS Minnie offset branch center={center:.6f} rad')
print(f'OLD shortest-zero method would jump by {jump_deg:.3f} output degrees at rotor 181deg')
