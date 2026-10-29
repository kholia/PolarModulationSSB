import sys, numpy as np
from timing_model import *
rate=int(__import__("os").environ.get("RATE","32000")); T=1e6/rate; F=14_200_000; SCL=2.0   # MHz -> bits per us
data=np.loadtxt(sys.argv[1]); amp=data[:,0]; ph=data[:,1]
N=len(ph); dt=0.25; fs=1e6/dt
d=ms_div_fw(F); P1,P2=pll(F,d); factor=C*d*rate/(2*np.pi*XTAL)
bd=bdiffs(ph,factor); L,regs=writes(P1,P2,bd)
ticks_nom=np.arange(N)*T
pat=np.array([31,31,31,32.0]); ticks_fw=np.concatenate([[0],np.cumsum(np.resize(pat,N-1))])
def bytes_of(p1,p2):
    return {2:(p1>>16)&3,3:(p1>>8)&255,4:p1&255,5:(p2>>16)&15,6:(p2>>8)&255,7:p2&255}
def events(ticks, mode, fixed=False):
    ev_t=[];ev_v=[]
    st=bytes_of(P1,P2)
    for k,(p1,p2,first) in enumerate(regs):
        if fixed: first=4
        nb=bytes_of(p1,p2); n=8-first
        if mode=="instant":
            ev_t.append(ticks[k]); st=nb
        elif mode=="stop":
            ev_t.append(ticks[k]+((2+n)*9+1)/SCL); st=nb
        if mode in("instant","stop"):
            ev_v.append(128*bd[k]); continue
        for i,r in enumerate(range(first,8)):    # per-byte latch
            st=dict(st); st[r]=nb[r]
            q1=(st[2]<<16)|(st[3]<<8)|st[4]; q2=(st[5]<<16)|(st[6]<<8)|st[7]
            ev_t.append(ticks[k]+(3+i)*9/SCL); ev_v.append((q1-P1)*C+(q2-P2))
    return np.array(ev_t),np.array(ev_v,dtype=float)
def phase_from(ev_t,ev_v):
    n=int(N*T/dt); idx=np.clip(np.round(ev_t/dt).astype(int),0,n)
    f=np.zeros(n); f_change=np.diff(np.concatenate([[0],ev_v]))
    np.add.at(f,idx[idx<n],f_change[idx<n]); f=np.cumsum(f)   # in p2 units
    # 128*b held for T gives b/factor rad  => rad per us = v/(128*factor*T)
    return np.cumsum(f)*dt/(128*factor*T)
win=None
def metrics(phi,shift_us):
    global win
    t=np.arange(len(phi))*dt
    a=np.interp(t-shift_us, ticks_nom+T, amp)          # a_k reached at end of its period
    x=a*np.exp(1j*phi)
    x=x[int(0.25e6/dt):]                                  # drop start-up
    x=x[: (len(x)//64)*64].reshape(-1,64).mean(1)        # decimate to 62.5 kHz
    if win is None or len(win)!=len(x): win=np.blackman(len(x))
    X=np.abs(np.fft.fftshift(np.fft.fft(x*win)))**2
    fr=np.fft.fftshift(np.fft.fftfreq(len(x),64*dt*1e-6))
    def band(lo,hi): return X[(fr>=lo)&(fr<hi)].sum()
    tone=max(band(690,710),band(1890,1910))
    imd3=max(band(-510,-490),band(3090,3110)); imd5=max(band(-1710,-1690),band(4290,4310))
    oob=band(-20000,200)+band(3000,20000)
    tot=X.sum()
    return 10*np.log10(imd3/tone), 10*np.log10(imd5/tone), 10*np.log10(oob/tot)
def best(phi):
    r=[(metrics(phi,s),s) for s in np.arange(-20,121,4.0)]
    return min(r,key=lambda z:z[0][2])
cases=[c for c in [("A ideal: uniform 31.25us ticks, instant update",ticks_nom,"instant",False),
       ("B firmware tick pattern 31/31/31/32, instant",ticks_fw,"instant",False),
       ("C fw ticks + update at STOP (variable length)",ticks_fw,"stop",False),
       ("D fw ticks + per-byte latch (variable length)",ticks_fw,"byte",False),
       ("E fw ticks + fixed 4-byte write, per-byte latch",ticks_fw,"byte",True),
       ("F fw ticks + fixed 4-byte write, update at STOP",ticks_fw,"stop",True)] if c[0][0] in (__import__("os").environ.get("CASES","ABCDEF"))]
print(f"L histogram data bytes: {dict(zip(*np.unique(L,return_counts=True)))}")
for name,tk,mode,fixed in cases:
    (i3,i5,oob),s=best(phase_from(*events(tk,mode,fixed)))
    print(f"{name:52s} IMD3 {i3:6.1f} dBc  IMD5 {i5:6.1f} dBc  out-of-band {oob:6.1f} dB  (best amp shift {s:+.0f} us)")

if __import__("os").environ.get("FIXED_SHIFTS"):
    # Evaluate case C (firmware timing) at explicit amplitude shifts (us); negative = amplitude early.
    phi=phase_from(*events(ticks_fw,"stop",False))
    for s in [float(v) for v in __import__("os").environ["FIXED_SHIFTS"].split(",")]:
        i3,i5,oob=metrics(phi,s)
        print(f"case C, amplitude shift {s:+6.1f} us: IMD3 {i3:6.1f} dBc  IMD5 {i5:6.1f} dBc  out-of-band {oob:6.1f} dB")
