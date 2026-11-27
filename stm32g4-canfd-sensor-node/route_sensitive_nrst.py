import math, pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
def balanced(s):
    d = q = e = 0
    for c in s:
        if q:
            if e: e = 0
            elif c == chr(92): e = 1
            elif c == chr(34): q = 0
        else:
            if c == chr(34): q = 1
            elif c == "(": d += 1
            elif c == ")":
                d -= 1
                if d < 0: return False
    return d == 0 and not q
assert balanced(txt), "Unbalanced input PCB"
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
net_map = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]+)\"\s*\)", txt):
    nid = int(m.group(1)); nn = m.group(2).strip()
    if nid > 0 and nn and nn not in net_map: net_map[nn] = nid
nrst_id = net_map.get("NRST", 0)
def rot(lx, ly, deg):
    r = math.radians(deg)
    return lx * math.cos(r) - ly * math.sin(r), lx * math.sin(r) + ly * math.cos(r)
targets = {"SW1": "2", "C17": "2", "U2": "7", "J2": "5"}
pins = []
for m in re.finditer(r"\(footprint\b", txt):
    fs = m.start(); fe_i = fe(txt, fs)
    if fe_i < 0: continue
    blk = txt[fs:fe_i+1]
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", blk)
    ref = rm.group(1).strip() if rm else ""
    if ref not in targets: continue
    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", blk)
    fx = float(am.group(1)) if am else 0.0
    fy = float(am.group(2)) if am else 0.0
    fd = float(am.group(3)) if (am and am.group(3)) else 0.0
    for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"", blk):
        pnum = pm.group(1).strip()
        if pnum != targets[ref]: continue
        ps = pm.start(); pe = fe(blk, ps)
        if pe < 0: continue
        pblk = blk[ps:pe+1]
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pblk)
        if pam:
            wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
            pins.append((ref, pnum, fx + wx, fy + wy))
order = ["SW1", "C17", "U2", "J2"]
pins.sort(key=lambda item: order.index(item[0]) if item[0] in order else 99)
segs = []
for idx in range(len(pins) - 1):
    _, _, x1, y1 = pins[idx]
    _, _, x2, y2 = pins[idx + 1]
    segs.append(f"  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width 0.1500) (layer \"F.Cu\") (net {nrst_id}))")
if segs:
    lp = txt.rfind(")")
    txt = txt[:lp] + "\n" + "\n".join(segs) + "\n)\n"
assert balanced(txt), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"ROUTE_NRST_OK | pins={len(pins)} | added_segments={len(segs)} | balanced={balanced(txt)}")
