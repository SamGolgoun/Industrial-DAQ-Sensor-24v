import math, pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
def bal(s):
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
assert bal(txt), "Unbalanced input PCB"
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
fp_boxes = []
targets = {"SW1": "2", "C17": "2", "U2": "7", "J2": "5"}
pins = {}
for m in re.finditer(r"\(footprint\b", txt):
    fs = m.start(); fe_i = fe(txt, fs)
    if fe_i < 0: continue
    blk = txt[fs:fe_i+1]
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", blk)
    ref = rm.group(1).strip() if rm else ""
    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", blk)
    fx = float(am.group(1)) if am else 0.0
    fy = float(am.group(2)) if am else 0.0
    fd = float(am.group(3)) if (am and am.group(3)) else 0.0
    rad = 3.2 if ref == "C19" else 2.5
    for pm in re.finditer(r"\(pad\b", blk):
        ps = pm.start(); pe = fe(blk, ps)
        if pe < 0: continue
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk[ps:pe+1])
        if pam:
            wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
            rad = max(rad, math.hypot(wx, wy) + (1.8 if ref == "C19" else 1.2))
    fp_boxes.append((ref, fx, fy, rad))
    if ref in targets:
        for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"", blk):
            if pm.group(1).strip() != targets[ref]: continue
            ps = pm.start(); pe = fe(blk, ps)
            if pe < 0: continue
            pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk[ps:pe+1])
            if pam:
                wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
                pins[ref] = (fx + wx, fy + wy)
def seg_hits(x1, y1, x2, y2, allow_refs):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    for ref, cx, cy, rad in fp_boxes:
        if ref in allow_refs or not ref: continue
        if l2 == 0: d = math.hypot(cx - x1, cy - y1)
        else:
            t = max(0.0, min(1.0, ((cx - x1) * dx + (cy - y1) * dy) / l2))
            d = math.hypot(cx - (x1 + t * dx), cy - (y1 + t * dy))
        if d < rad: return True
    return False
kept_lines = []
for line in txt.splitlines():
    if line.lstrip().startswith("(segment ") and re.search(rf"\(\s*net\s+{nrst_id}\s*\)", line):
        continue
    kept_lines.append(line)
txt = "\n".join(kept_lines) + "\n"
new_segs = []
ordered_refs = [r for r in ["SW1", "C17", "U2", "J2"] if r in pins]
for i in range(len(ordered_refs) - 1):
    r1, r2 = ordered_refs[i], ordered_refs[i + 1]
    x1, y1 = pins[r1]; x2, y2 = pins[r2]
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    if dist < 1e-4: continue
    ux, uy = dx / dist, dy / dist
    nx, ny = -uy, ux
    d1 = min(2.2, dist * 0.2)
    d2 = min(dist * 0.5, d1 + 2.5)
    p_exit = (x1 + ux * d1, y1 + uy * d1)
    p_mid = (x1 + ux * d2, y1 + uy * d2)
    cand_list = [(x1, y1), p_exit, p_mid, (x2, y2)]
    if seg_hits(x1, y1, x2, y2, {r1, r2}) or any(seg_hits(cand_list[k][0], cand_list[k][1], cand_list[k+1][0], cand_list[k+1][1], {r1, r2}) for k in range(3)):
        for off in (4.8, 6.2, -4.8, -6.2):
            p_fan = (p_exit[0] + ux * 2.2 + nx * off * 0.65, p_exit[1] + uy * 2.2 + ny * off * 0.65)
            p_byp = (p_mid[0] + nx * off, p_mid[1] + ny * off)
            p_ret = (x2 - ux * min(2.5, dist * 0.2) + nx * (off * 0.35), y2 - uy * min(2.5, dist * 0.2) + ny * (off * 0.35))
            trial = [(x1, y1), p_exit, p_fan, p_byp, p_ret, (x2, y2)]
            if not any(seg_hits(trial[k][0], trial[k][1], trial[k+1][0], trial[k+1][1], {r1, r2}) for k in range(len(trial) - 1)):
                cand_list = trial
                break
    for k in range(len(cand_list) - 1):
        w = 0.1500 if k == 0 else 0.3000
        new_segs.append(f"  (segment (start {cand_list[k][0]:.4f} {cand_list[k][1]:.4f}) (end {cand_list[k+1][0]:.4f} {cand_list[k+1][1]:.4f}) (width {w:.4f}) (layer \"F.Cu\") (net {nrst_id}))")
if new_segs:
    lp = txt.rfind(")")
    txt = txt[:lp] + "\n" + "\n".join(new_segs) + "\n)\n"
assert bal(txt), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"CLEAN_ANGLES_NRST_OK | segments_added={len(new_segs)} | balanced={bal(txt)}")
