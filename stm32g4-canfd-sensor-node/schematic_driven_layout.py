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
xs=[float(m.group(1)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts",txt)]
ys=[float(m.group(2)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts",txt)]
x0=min(xs) if xs else 100.0; y0=min(ys) if ys else 60.0; x1=x0+80.0; y1=y0+60.0
parsed=[]
for s,e,b in fps:
 rm=re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"",b)
 ref=rm.group(1).strip() if rm else ""
 am=re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)",b)
 if not am:continue
 fx,fy=float(am.group(1)),float(am.group(2)); fdeg=float(am.group(3)) if am.group(3) else 0.0
 pads=[]
 for m in re.finditer(r"\(pad\s+\"([^\"]*)\"",b):
  ps=m.start(); pe=fend(b,ps)
  if pe<0:continue
  pb=b[ps:pe+1]
  pm=re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)",pb)
  nm=re.search(r"\(\s*net\s+\d+\s+\"([^\"]*)\"",pb)
  if pm:pads.append((float(pm.group(1)),float(pm.group(2)),nm.group(1).strip() if nm else ""))
 parsed.append({"s":s,"e":e,"b":b,"am":am,"ref":ref,"x":fx,"y":fy,"deg":fdeg,"pads":pads})
by_ref={f["ref"]:f for f in parsed if f["ref"]}
# Schematic-driven functional clusters with clean routing gaps
target_pos={
 "H1":(x0+5.0,y0+5.0),"H2":(x1-5.0,y0+5.0),"H3":(x1-5.0,y1-5.0),"H4":(x0+5.0,y1-5.0),
 "FID1":(x0+13.5,y0+7.5),"FID2":(x1-13.5,y0+7.5),"FID3":(x1-13.5,y1-7.5),
 # 24V input protection & TPS54302 buck stage (top-left noisy zone)
 "J2":(x0+11.5,y0+15.0),"F1":(x0+16.5,y0+12.5),"D2":(x0+21.5,y0+12.5),"Q1":(x0+17.5,y0+18.5),
 "D1":(x0+23.0,y0+18.5),"R1":(x0+17.0,y0+23.5),"R2":(x0+21.5,y0+23.5),"R4":(x0+26.0,y0+23.5),
 "U1":(x0+31.0,y0+15.5),"C3":(x0+26.5,y0+13.0),"C20":(x0+26.5,y0+17.5),"C6":(x0+35.5,y0+12.0),
 "L1":(x0+36.5,y0+17.5),"D3":(x0+32.0,y0+21.5),"C8":(x0+41.5,y0+14.0),"C12":(x0+41.5,y0+18.5),
 "C14":(x0+41.5,y0+23.0),"C16":(x0+36.5,y0+23.5),"R5":(x0+31.5,y0+26.0),"R10":(x0+36.5,y0+27.5),
 # 4-20mA input parallel RC/Zener filter cluster (bottom-left)
 "J4":(x0+12.0,y1-14.0),"R7":(x0+18.5,y1-17.0),"D4":(x0+18.5,y1-12.5),"C22":(x0+23.5,y1-14.5),
 # Analog +3V3_ANA filter & reset cluster (mid-left)
 "FB1":(x0+14.0,y0+30.5),"C18":(x0+19.0,y0+30.5),"C21":(x0+24.0,y0+30.5),
 "SW1":(x0+14.0,y0+37.0),"C17":(x0+20.5,y0+37.0),
 # MCU STM32G4 & Crystal Y1 with load caps C13/C19 & decoupling bank (center-right sensitive zone)
 "U2":(x0+54.0,y0+31.0),"Y1":(x0+46.5,y0+27.5),"C13":(x0+46.0,y0+23.0),"C19":(x0+46.0,y0+32.0),
 "C4":(x0+50.5,y0+21.5),"C5":(x0+55.0,y0+21.5),"C7":(x0+59.5,y0+21.5),
 "C9":(x0+50.5,y0+40.5),"C11":(x0+55.0,y0+40.5),"C24":(x0+59.5,y0+40.5),
 # CAN-FD transceiver U3 + 120R termination R6 + C1 + J1 (top-right)
 "U3":(x1-17.5,y0+15.0),"R6":(x1-12.0,y0+13.0),"C1":(x1-22.0,y0+13.0),"J1":(x1-11.5,y0+19.0),
 # Sensors TMP101 (U4) + I2C pull-ups R9/R11 + C23 & LIS3DH (U5) + C10/C15 (bottom-right)
 "U4":(x0+35.0,y1-15.5),"C23":(x0+35.0,y1-10.5),"R9":(x0+40.5,y1-17.0),"R11":(x0+40.5,y1-12.5),
 "U5":(x1-16.5,y1-16.0),"C10":(x1-22.0,y1-17.5),"C15":(x1-22.0,y1-13.0),"R3":(x0+47.5,y1-12.5),"R8":(x0+47.5,y1-17.0)
}
moved=[]
for ref,(tx,ty) in target_pos.items():
 if ref in by_ref:
  f=by_ref[ref]
  if abs(f["x"]-tx)>0.01 or abs(f["y"]-ty)>0.01:
   moved.append((ref,round(f["x"],2),round(f["y"],2),round(tx,2),round(ty,2)))
   f["x"],f["y"]=round(tx,3),round(ty,3)
# 4x90 degree rotation optimization for C, R, D
def rot(lx,ly,d):
 r=math.radians(d); return lx*math.cos(r)-ly*math.sin(r), lx*math.sin(r)+ly*math.cos(r)
def score(fp,d):
 p0=(fp["x"],fp["y"]); tot=0.0
 for lx,ly,net in fp["pads"]:
  wx,wy=rot(lx,ly,d); pt=(p0[0]+wx,p0[1]+wy)
  cands=[]
  if net:
   cands=[(o["x"]+rot(olx,oly,o["deg"])[0],o["y"]+rot(olx,oly,o["deg"])[1]) for o in parsed if o["ref"]!=fp["ref"] for olx,oly,onet in o["pads"] if onet==net]
  if not cands:
   cands=[(o["x"],o["y"]) for o in parsed if o["ref"]!=fp["ref"] and math.hypot(o["x"]-p0[0],o["y"]-p0[1])<12.0]
  if cands:tot+=min(math.hypot(pt[0]-q[0],pt[1]-q[1]) for q in cands)
 return tot
rotated=[]
for f in parsed:
 if f["ref"].startswith(("C","R","D")):
  cur=float(f["deg"]%360.0); best_s=score(f,cur); best_d=cur
  for k in range(4):
   c=(cur+k*90.0)%360.0; sc=score(f,c)
   if sc+0.01<best_s:best_s,best_d=sc,c
  if abs(best_d-cur)>0.01:
   rotated.append((f["ref"],int(cur),int(best_d)))
   f["deg"]=best_d
edits=[]
for f in parsed:
 am=f["am"]; dv=float(f["deg"]%360.0); ds=int(dv) if dv.is_integer() else round(dv,3)
 nat=f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f} {ds})" if ds!=0 else f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f})"
 edits.append((f["s"],f["e"],f["b"][:am.start()]+nat+f["b"][am.end():]))
for s,e,nb in sorted(edits,key=lambda x:x[0],reverse=True):
 txt=txt[:s]+nb+txt[e:]
diff=txt.count("(")-txt.count(")")
if diff>0:txt=txt.rstrip()+"\n"+(")"*diff)+"\n"
PCB.write_text(txt,encoding="utf-8")
print(f"SCH_DRIVEN_OK | moved={len(moved)} | rotated={len(rotated)} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
for m in moved[:25]:print(f"  MOVE {m[0]}: ({m[1]},{m[2]}) -> ({m[3]},{m[4]})")
for r in rotated[:25]:print(f"  ROT  {r[0]}: {r[1]}deg -> {r[2]}deg")
