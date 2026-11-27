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
# Remove prior generated segments if present
txt = re.sub(r"\n  \(segment \(start [^\n]*?\(width 0\.2500\) \(layer \"F\.Cu\"\) \(net \d+\) \(uuid \"[^\"]+\"\)\)", "", txt)
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
seen = set(); segs = []
for nn, pts in pads_by_net.items():
    if len(pts) < 2: continue
    for a in range(len(pts)):
        best_j = None; best_d = 999.0
        for b in range(len(pts)):
            if a == b: continue
            d = math.hypot(pts[a][0]-pts[b][0], pts[a][1]-pts[b][1])
            if 0.3 < d <= 18.0 and d < best_d:
                best_d = d; best_j = b
        if best_j is not None:
            u, v = min(a, best_j), max(a, best_j)
            key = (nn, round(pts[u][0],2), round(pts[u][1],2), round(pts[v][0],2), round(pts[v][1],2))
            if key not in seen:
                seen.add(key)
                x1, y1 = pts[u]; x2, y2 = pts[v]
                segs.append(f"  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width 0.2500) (layer \"F.Cu\") (net {nn}) (uuid \"{uuid.uuid4()}\"))")
if segs:
    er2 = fe(txt, 0)
    txt = txt[:er2] + "\n" + "\n".join(segs) + "\n" + txt[er2:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
has_zones = "(zone " in txt
print(f"AUTO_TRACKS_OK | segments={len(segs)} | has_zones={has_zones} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
if has_zones:
    print("ACTION_REQUIRED: Open PCB in KiCad and press B to refill final copper zones.")
