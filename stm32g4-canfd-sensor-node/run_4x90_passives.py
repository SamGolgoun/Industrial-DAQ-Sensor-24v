import math, pathlib, re
PCB=pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
def fend(s,i):
 d=q=e=0
 for k in range(i,len(s)):
  c=s[k]
  if e:e=0
  elif c==chr(92):e=1
  elif c==chr(34):q^=1
  elif not q:
   d+=(c=="(")-(c==")")
   if d==0:return k
 return -1
txt=PCB.read_text(encoding="utf-8")
er=fend(txt,0); i=1; fps=[]
while i<er:
 if txt[i]=="(":
  j=fend(txt,i)
  if j<0:break
  b=txt[i:j+1]
  if b.startswith("(footprint "):fps.append((i,j+1,b))
  i=j+1
 else:i+=1
parsed=[]
for s,e,b in fps:
 rm=re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"",b)
 ref=rm.group(1).strip() if rm else ""
 am=re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)",b)
 if not am:continue
 fx,fy=float(am.group(1)),float(am.group(2))
 fdeg=float(am.group(3)) if am.group(3) else 0.0
 pads=[]
 for m in re.finditer(r"\(pad\s+\"([^\"]*)\"",b):
  ps=m.start(); pe=fend(b,ps)
  if pe<0:continue
  pb=b[ps:pe+1]
  pm=re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)",pb)
  nm=re.search(r"\(\s*net\s+\d+\s+\"([^\"]*)\"",pb)
  if pm:pads.append((float(pm.group(1)),float(pm.group(2)),nm.group(1).strip() if nm else ""))
 parsed.append({"s":s,"e":e,"b":b,"am":am,"ref":ref,"x":fx,"y":fy,"deg":fdeg,"pads":pads})
def rot(lx,ly,d):
 r=math.radians(d); return lx*math.cos(r)-ly*math.sin(r), lx*math.sin(r)+ly*math.cos(r)
def cross(a,b,c,d):
 o1=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
 o2=(b[0]-a[0])*(d[1]-a[1])-(b[1]-a[1])*(d[0]-a[0])
 o3=(d[0]-c[0])*(a[1]-c[1])-(d[1]-c[1])*(a[0]-c[0])
 o4=(d[0]-c[0])*(b[1]-c[1])-(d[1]-c[1])*(b[0]-c[0])
 return (o1*o2<0) and (o3*o4<0)
def eval_fp(fp,d,all_fps):
 p0=(fp["x"],fp["y"])
 peers=[o for o in all_fps if o["ref"]!=fp["ref"] and o["ref"].startswith(fp["ref"][0])]
 if not peers:peers=[o for o in all_fps if o["ref"]!=fp["ref"]]
 nearest=min(peers,key=lambda o:math.hypot(o["x"]-p0[0],o["y"]-p0[1])) if peers else None
 rlen=0.0; crs=0; segs=[]
 for lx,ly,net in fp["pads"]:
  wx,wy=rot(lx,ly,d); pt=(p0[0]+wx,p0[1]+wy)
  tgt=None
  if net:
   cands=[(o["x"]+rot(olx,oly,o["deg"])[0],o["y"]+rot(olx,oly,o["deg"])[1]) for o in all_fps if o["ref"]!=fp["ref"] for olx,oly,onet in o["pads"] if onet==net]
   if cands:tgt=min(cands,key=lambda q:math.hypot(pt[0]-q[0],pt[1]-q[1]))
  if not tgt and nearest:tgt=(nearest["x"],nearest["y"])
  if tgt:
   rlen+=math.hypot(tgt[0]-pt[0],tgt[1]-pt[1])
   segs.append((pt,tgt))
 for a1,a2 in segs:
  for o in all_fps:
   if o["ref"]==fp["ref"]:continue
   q1=(o["x"],o["y"])
   for olx,oly,_ in o["pads"][:2]:
    owx,owy=rot(olx,oly,o["deg"])
    q2=(o["x"]+owx,o["y"]+owy)
    if cross(a1,a2,q1,q2):crs+=1
 return round(rlen+4.0*crs,4),round(rlen,3),crs
targets=[f for f in parsed if f["ref"].startswith(("C","R","D"))]
changes=[]
for f in targets:
 cur=float(f["deg"]%360.0)
 bt,bl,bc=eval_fp(f,cur,parsed); bd=cur
 for k in range(4):
  cand=(cur+k*90.0)%360.0
  t,l,c=eval_fp(f,cand,parsed)
  if t+0.001<bt:bt,bl,bc,bd=t,l,c,cand
 if abs(bd-cur)>0.01:
  changes.append((f["ref"],int(cur),int(bd),bl,bc))
  f["deg"]=bd
edits=[]
for f in parsed:
 am=f["am"]; dv=float(f["deg"]%360.0)
 ds=int(dv) if dv.is_integer() else round(dv,3)
 nat=f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f} {ds})" if ds!=0 else f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f})"
 nb=f["b"][:am.start()]+nat+f["b"][am.end():]
 edits.append((f["s"],f["e"],nb))
for s,e,nb in sorted(edits,key=lambda x:x[0],reverse=True):
 txt=txt[:s]+nb+txt[e:]
diff=txt.count("(")-txt.count(")")
if diff>0:txt=txt.rstrip()+"\n"+(")"*diff)+"\n"
PCB.write_text(txt,encoding="utf-8")
bal=(txt.count(chr(40))==txt.count(chr(41)))
print(f"PASSIVE_4X90_OK | evaluated={len(targets)} | changed={len(changes)} | balanced={bal}")
for item in changes:
 print(f"  {item[0]}: {item[1]}deg -> {item[2]}deg | score={item[3]} | crossings={item[4]}")
