import numpy as np, json
def load_dat(f):
    T,V=[],[]
    for l in open(f):
        r=l.split()
        if len(r)<34 or not r[0][:2]=='20': continue
        try:
            h,m,s=r[1].split(':'); T.append(float(h)*3600+float(m)*60+float(s)); V.append([float(x) for x in r[2:34]])
        except ValueError: continue
    return np.array(T),np.array(V)
def load_jsonl(f):
    W,S=[],[]
    for l in open(f):
        d=json.loads(l)
        if d.get('k')=='wsk': W.append(d)
        elif d.get('k')=='status': S.append(d)
    return W,S
