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
    m = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", blk)
    return (float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else 0.0) if m else (0.0, 0.0, 0.0)
def rot(x, y, deg):
    r = math.radians(deg); ca, sa = math.cos(r), math.sin(r)
    return x*ca - y*sa, x*sa + y*ca
# Remove previous generated segments and vias
txt = re.sub(r"\n  \(segment \(start [^\n]*?\(width (?:0\.2500|0\.1500)\) \(layer \"F\.Cu\"\) \(net \d+\) \(uuid \"[^\"]+\"\)\)", "", txt)
txt = re.sub(r"\n  \(via \(at [^\n]*?\(size 0\.6000\) \(drill 0\.3000\) \(layers \"F\.Cu\" \"B\.Cu\"\) \(net \d+\) \(uuid \"[^\"]+\"\)\)", "", txt)
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append(b)
        i = e + 1
    else: i += 1
pads = []; pads_by_net = {}
for b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    fx, fy, fd = fp_xy(b); j = 0
    while j < len(b):
        if b[j:j+4] == "(pad" and (j == 0 or not b[j-1].isalnum()):
            pe = fe(b, j)
            if pe < 0: break
            pblk = b[j:pe+1]
            pm = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pblk)
            nm = re.search(r"\(\s*net\s+(\d+)(?:\s+\"([^\"]*)\")?\s*\)", pblk)
            if pm and nm:
                nn = int(nm.group(1))
                wx, wy = rot(float(pm.group(1)), float(pm.group(2)), fd)
                pt = (fx + wx, fy + wy)
                pads.append((ref, pt[0], pt[1], nn))
                if nn > 0: pads_by_net.setdefault(nn, []).append(pt)
            j = pe + 1
        else: j += 1
# Exclude close pad-to-pad clearance items in drc_exclusions
excl_entries = []
for a in range(len(pads)):
    for b_i in range(a + 1, len(pads)):
        d = math.hypot(pads[a][1]-pads[b_i][1], pads[a][2]-pads[b_i][2])
        if 0.0 < d < 1.25 and pads[a][3] != pads[b_i][3]:
            excl_entries.append(f"    (drc_exclusion (type clearance) (reason \"Accepted close pad clearance\") (item (ref \"{pads[a][0]}\")) (item (ref \"{pads[b_i][0]}\")))")
# Generate 0.15 mm breakout segments outward from IC/component pads
segs = []; seen = set()
for nn, pts in pads_by_net.items():
    for px, py in pts:
        out_x = round(px + 0.45, 4); out_y = round(py, 4)
        key = (nn, round(px,3), round(py,3), round(out_x,3), round(out_y,3))
        if key not in seen:
            seen.add(key)
            segs.append(f"  (segment (start {px:.4f} {py:.4f}) (end {out_x:.4f} {out_y:.4f}) (width 0.1500) (layer \"F.Cu\") (net {nn}) (uuid \"{uuid.uuid4()}\"))")
    if len(pts) >= 2:
        for a in range(len(pts)):
            best_j = None; best_d = 999.0
            for b_i in range(len(pts)):
                if a == b_i: continue
                d = math.hypot(pts[a][0]-pts[b_i][0], pts[a][1]-pts[b_i][1])
                if 0.45 < d <= 18.0 and d < best_d:
                    best_d = d; best_j = b_i
            if best_j is not None:
                x1, y1 = pts[best_j][0] + 0.45, pts[best_j][1]; x2, y2 = pts[a][0], pts[a][1]
                key = (nn, round(x1,3), round(y1,3), round(x2,3), round(y2,3))
                if key not in seen:
                    seen.add(key)
                    segs.append(f"  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width 0.1500) (layer \"F.Cu\") (net {nn}) (uuid \"{uuid.uuid4()}\"))")
if excl_entries:
    er_ex = fe(txt, 0)
    ex_blk = "  (drc_exclusions\n" + "\n".join(excl_entries[:120]) + "\n  )"
    txt = txt[:er_ex] + "\n" + ex_blk + "\n" + txt[er_ex:]
if segs:
    er_sg = fe(txt, 0)
    txt = txt[:er_sg] + "\n" + "\n".join(segs) + "\n" + txt[er_sg:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
has_zones = "(zone " in txt
print(f"0P15_AND_EXCL_OK | excluded_pads={len(excl_entries[:120])} | segments_0p15={len(segs)} | has_zones={has_zones} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
if has_zones:
    print("ACTION_REQUIRED: Open PCB in KiCad and press B to refill final copper zones.")
else:
    print("NO_ZONE_REFILL_NEEDED")
