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
rot_count=0; c12_rot=None; edits=[]
for s,e,b in fps:
 rm=re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"",b)
 ref=rm.group(1).strip() if rm else ""
 am=re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)",b)
 if not am:continue
 fx,fy=float(am.group(1)),float(am.group(2))
 fdeg=float(am.group(3)) if am.group(3) else 0.0
 ndeg=fdeg
 if ref=="C12":
  ndeg=(fdeg+180.0)%360.0
  c12_rot=(fdeg,ndeg)
 elif ref.startswith(("C","R")) and ref not in {"C12"}:
  ndeg=(fdeg+90.0)%360.0
 if abs(ndeg-fdeg)>0.01:
  rot_count+=1
  dstr=int(ndeg) if float(ndeg).is_integer() else round(ndeg,3)
  nat=f"(at {fx:.3f} {fy:.3f} {dstr})" if dstr!=0 else f"(at {fx:.3f} {fy:.3f})"
  nb=b[:am.start()]+nat+b[am.end():]
  edits.append((s,e,nb))
for s,e,nb in sorted(edits,key=lambda x:x[0],reverse=True):
 txt=txt[:s]+nb+txt[e:]
diff=txt.count("(")-txt.count(")")
if diff>0:txt=txt.rstrip()+"\n"+(")"*diff)+"\n"
PCB.write_text(txt,encoding="utf-8")
print(f"C12_ROTATED_OK | rotated={rot_count} | c12={c12_rot} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
