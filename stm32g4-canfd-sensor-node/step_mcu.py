import math, pathlib, re

pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = pcb.read_text(encoding="utf-8")
net_ids = {m.group(2): int(m.group(1)) for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt)}

def find_end(s, idx):
d = q = esc = 0
for i in range(idx, len(s)):
    c = s[i]
    if esc: esc = 0
    elif c == "\\": esc = 1
    elif c == "\"\"": q ^= 1
    elif not q:
        d += (c == "(") - (c == ")")
        if d == 0: return i
return -1

def parse_fps(s):
er = find_end(s, 0); i = 1; out = []
while i < er:
    if s[i] == "(":
        e = find_end(s, i)
        if e < 0: break
        b = s[i:e+1]
        if b.startswith("(footprint "):
            rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
            ref = rm.group(1).strip() if rm else ""
            am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", b[:1500])
            fx, fy, fd = (float(am.group(1)), float(am.group(2)), float(am.group(3) or 0)) if am else (0.0, 0.0, 0.0)
            pads = []
            for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"[\s\S]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[\s\S]*?\(size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[\s\S]*?\(net\s+\"([^\"]*)\"\)", b):
                nn = pm.group(6)
                pads.append({"num": pm.group(1), "lx": float(pm.group(2)), "ly": float(pm.group(3)), "r": max(float(pm.group(4)), float(pm.group(5)))*0.5, "net": nn, "code": net_ids.get(nn, 0)})
            out.append({"ref": ref, "x": fx, "y": fy, "deg": fd, "pads": pads, "blk": b})
        i = e + 1
    else: i += 1
return out

def pxy(fp, pad):
rad = math.radians(-fp["deg"])
return fp["x"] + pad["lx"]*math.cos(rad) - pad["ly"]*math.sin(rad), fp["y"] + pad["lx"]*math.sin(rad) + pad["ly"]*math.cos(rad)

def pt_seg(px, py, x1, y1, x2, y2):
dx, dy = x2-x1, y2-y1; l2 = dx*dx + dy*dy
if l2 == 0: return math.hypot(px-x1, py-y1)
t = max(0.0, min(1.0, ((px-x1)*dx + (py-y1)*dy)/l2))
return math.hypot(px-(x1+t*dx), py-(y1+t*dy))

fps = parse_fps(txt)
u1 = next((f for f in fps if f["ref"] == "U1"), None) or next((f for f in fps if "STM32" in f["blk"].upper()), None)
all_pads = [{"ref": f["ref"], "num": p["num"], "x": pxy(f, p)[0], "y": pxy(f, p)[1], "r": p["r"], "net": p["net"], "code": p["code"]} for f in fps for p in f["pads"]]

segs = []
routed = 0
if u1:
for up in [p for p in all_pads if p["ref"] == u1["ref"] and p["net"] and p["net"] != "GND"]:
    tgs = [p for p in all_pads if p["net"] == up["net"] and not (p["ref"] == u1["ref"] and p["num"] == up["num"])]
    if not tgs: continue
    near = min(tgs, key=lambda tp: math.hypot(tp["x"]-up["x"], tp["y"]-up["y"]))
    x1, y1, x2, y2 = up["x"], up["y"], near["x"], near["y"]
    if math.hypot(x2-x1, y2-y1) < 0.25: continue
    vx, vy = x1 - u1["x"], y1 - u1["y"]
    edx, edy = ((1.0 if vx >= 0 else -1.0), 0.0) if abs(vx) >= abs(vy) else (0.0, (1.0 if vy >= 0 else -1.0))
    nx1, ny1 = round(x1 + edx*0.65, 4), round(y1 + edy*0.65, 4)
    wide = near["ref"].startswith(("R", "D", "L", "FB", "F")) or ("3V3" in up["net"])
    tw = 0.35 if wide else 0.25
    ew = 0.40 if (wide and near["r"] >= 0.35) else 0.25
    dx, dy = x2 - nx1, y2 - ny1; diag = min(abs(dx), abs(dy))
    sx, sy = (1.0 if dx >= 0 else -1.0), (1.0 if dy >= 0 else -1.0)
    mx1, my1 = round(nx1 + sx*diag, 4), round(ny1 + sy*diag, 4)
    cand = [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, mx1, my1, tw), (mx1, my1, x2, y2, ew)]
    cc = [(a, b, e, f, w) for a, b, e, f, w in cand if math.hypot(e-a, f-b) > 0.03]
    if cc and all(pt_seg(pd["x"], pd["y"], a, b, e, f) >= (pd["r"] + w*0.5 + 0.14) for a, b, e, f, w in cc for pd in all_pads if pd["net"] != up["net"]):
        segs.extend((a, b, e, f, w, up["code"]) for a, b, e, f, w in cc)
        routed += 1

gnd_code = net_ids.get("GND", 0)
sens = [(f["x"], f["y"]) for f in fps if f["ref"].startswith("Y") or any(k in p["net"].upper() for p in f["pads"] for k in ("OSC", "ADC", "ANA"))]
vias = []
for sx_c, sy_c in sens[:8]:
for k in range(10):
    ang = 2.0 * math.pi * k / 10.0
    vx, vy = round(sx_c + 3.2*math.cos(ang), 4), round(sy_c + 3.2*math.sin(ang), 4)
    if all(pd["net"] == "GND" or math.hypot(vx-pd["x"], vy-pd["y"]) >= (pd["r"] + 0.55) for pd in all_pads):
        vias.append((vx, vy, 0.60, 0.30, gnd_code))

lines = [f"  (segment (start {a:.4f} {b:.4f}) (end {e:.4f} {f:.4f}) (width {w:.2f}) (layer \"F.Cu\") (net {nc}))" for a, b, e, f, w, nc in segs]
lines += [f"  (via (at {vx:.4f} {vy:.4f}) (size {vs:.2f}) (drill {vd:.2f}) (layers \"F.Cu\" \"In1.Cu\" \"In2.Cu\" \"B.Cu\") (net {nc}))" for vx, vy, vs, vd, nc in vias]
if lines:
lp = txt.rfind(")")
pcb.write_text(txt[:lp] + "\n" + "\n".join(lines) + "\n)\n", encoding="utf-8")
print("MCU found:", bool(u1), "Routed nets:", routed, "Segments:", len(segs), "GND fence vias:", len(vias))
