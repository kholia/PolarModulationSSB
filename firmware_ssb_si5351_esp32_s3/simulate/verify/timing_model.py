import sys, numpy as np
C = 1048575; XTAL = 25_000_000
def ms_div_fw(f):              # firmware: PLL fraction b/c near 1/2
    best=None
    for d in range(6,255,2):
        v=f*d
        if not 600e6<=v<=900e6: continue
        r=v%XTAL; fe=abs(2*r-XTAL); ve=abs(v-800_000_000)
        k=(fe,ve)
        if best is None or k<best[0]: best=(k,d)
    return best[1]
def ms_div_p2(f):              # alternative: P2 = 128*b mod c near c/2
    best=None
    for d in range(6,255,2):
        v=f*d
        if not 600e6<=v<=900e6: continue
        b=((v%XTAL)*C)//XTAL; p2=(128*b)%C
        k=(abs(2*p2-C),abs(v-800_000_000))
        if best is None or k<best[0]: best=(k,d)
    return best[1]
def pll(f,d):
    v=f*d; a=v//XTAL; b=((v%XTAL)*C)//XTAL
    return 128*a+(128*b)//C-512, (128*b)%C
def bdiffs(phase, factor):
    acc=0.0; out=np.empty(len(phase),dtype=np.int64)
    for i,p in enumerate(phase):
        acc+=np.float32(p)*np.float32(factor); b=int(acc); acc-=b; out[i]=b
    return out
def writes(p1b,p2b,bd):
    lp1,lp2=p1b,p2b; L=[];regs=[]
    for b in bd:
        p2=p2b+128*int(b); adj=0
        while p2>=C: p2-=C; adj+=1
        while p2<0: p2+=C; adj-=1
        p1=p1b+adj
        if (p1&~0xff)!=(lp1&~0xff): n=6; first=2
        elif (p1&0xff)!=(lp1&0xff): n=4; first=4
        elif (p2>>8)==(lp2>>8): n=1; first=7
        elif (p2>>16)==(lp2>>16): n=2; first=6
        else: n=3; first=5
        L.append(n); regs.append((p1,p2,first)); lp1,lp2=p1,p2
    return np.array(L),regs
if __name__=="__main__":
    f=int(sys.argv[2]) if len(sys.argv)>2 else 14_200_000
    data=np.loadtxt(sys.argv[1]); rate=int(sys.argv[3]) if len(sys.argv)>3 else 32000
    for name,fn in (("firmware ms_div",ms_div_fw),("p2-centred ms_div",ms_div_p2)):
        d=fn(f); p1,p2=pll(f,d)
        factor=C*d*rate/(2*np.pi*XTAL)
        bd=bdiffs(data[:,1],factor); L,_=writes(p1,p2,bd)
        h={n:np.mean(L==n)*100 for n in (1,2,3,4,6)}
        print(f"{f/1e6:.3f} MHz {name}: div={d} base P2={p2} ({p2/C:.3f} of c) "
              f"b_diff[{bd.min()},{bd.max()}]  data-bytes %: "+" ".join(f"{k}:{v:.1f}" for k,v in h.items()))
