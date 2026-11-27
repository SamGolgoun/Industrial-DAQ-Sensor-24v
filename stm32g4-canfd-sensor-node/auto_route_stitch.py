import math, pathlib, re, uuid
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
assert txt.count("(") == txt.count(")"), "Unbalanced PCB"
def fe(s, idx):
    d = q = e = 0
    for i in range(idx, len(s)):
        c = s[i]
        if e: e = 0
        elif c == chr(92): e = 1
        elif c == chr(34): q ^= 1
        elif not q:
            d += (c == "(") - (c == ")")
            if d == 0: return i
    return -1
def fp_xy(blk):
    i = 1; d = 1; q = e = 0
    while i < len(blk):
        c = blk[i]
        if e: e = 0
        elif c == chr(92): e = 1
        elif c == chr(34): q ^= 1
        elif not q:
            if c == "(":
                if d == 1 and re.match(r"^\(\s*at\b", blk[i:i+10]):
                    ae = fe(blk, i)
                    m = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", blk[i:ae+1])
                    if m: return float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else 0.0
                d += 1
            elif c == ")": d -= 1
        i += 1
    return 0.0, 0.0, 0.0
def rot(x, y, deg):
    r = math.radians(deg); ca, sa = math.cos(r), math.sin(r)
    return x*ca - y*sa, x*sa + y*ca
def find_net_num(s, name):
    for m in re.finditer(rf"\(\s*net\s+(\d+)\s+\"{re.escape(name)}\"\s*\)", s):
        return int(m.group(1))
    return None
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append(b)
        i = e + 1
    else: i += 1
pads_by_net = {}
for b in fps:
    fx, fy, fd = fp_xy(b)
    for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+-?\d+(?:\.\d+)?)?\)[^\n]*?\(net\s+(\d+)\s+\"([^\"]*)\"\)", b):
        lx, ly = float(m.group(1)), float(m.group(2))
        wx, wy = rot(lx, ly, fd)
        nn = int(m.group(3))
        if nn > 0: pads_by_net.setdefault(nn, []).append((fx+wx, fy+wy))
added_segs = []
for nn, pts in pads_by_net.items():
    if len(pts) < 2: continue
    for idx_a in range(len(pts)):
        best_j = None; best_d = 999.0
        for idx_b in range(len(pts)):
            if idx_a == idx_b: continue
            d = math.hypot(pts[idx_a][0]-pts[idx_b][0], pts[idx_a][1]-pts[idx_b][1])
            if 0.4 < d <= 8.5 and d < best_d:
                best_d = d; best_j = idx_b
        if best_j is not None and idx_a < best_j:
            x1, y1 = pts[idx_a]; x2, y2 = pts[best_j]
            seg = f"  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width 0.2500) (layer \"F.Cu\") (net {nn}) (uuid \"{uuid.uuid4()}\"))"
            added_segs.append(seg)
gnd_net = find_net_num(txt, "GND") or find_net_num(txt, "/GND") or 1
xs = [float(m.group(1)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
ys = [float(m.group(2)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
x0 = (min(xs) if xs else 100.0) + 6.0; y0 = (min(ys) if ys else 60.0) + 6.0
added_vias = []
for r_i in range(4):
    for c_i in range(5):
        vx = round(x0 + c_i * 15.0, 4); vy = round(y0 + r_i * 13.0, 4)
        via = f"  (via (at {vx:.4f} {vy:.4f}) (size 0.6000) (drill 0.3000) (layers \"F.Cu\" \"B.Cu\") (net {gnd_net}) (uuid \"{uuid.uuid4()}\"))"
        added_vias.append(via)
insert_blk = "\n".join(added_segs + added_vias)
if insert_blk:
    txt = txt[:er] + "\n" + insert_blk + "\n" + txt[er:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
has_zones = "(zone " in txt
print(f"AUTO_ROUTE_STITCH_OK | segments={len(added_segs)} | vias={len(added_vias)} | has_zones={has_zones} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
if has_zones:
    print("ACTION_REQUIRED: Open PCB in KiCad and press B to refill copper zones now.")
else:
    print("NO_ZONE_REFILL_NEEDED")
