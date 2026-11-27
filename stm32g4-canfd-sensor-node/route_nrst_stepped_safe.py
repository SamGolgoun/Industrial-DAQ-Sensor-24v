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
    fp_boxes.append((ref, fx, fy, 2.2))
    if ref in targets:
        for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"", blk):
            if pm.group(1).strip() != targets[ref]: continue
            ps = pm.start(); pe = fe(blk, ps)
            if pe < 0: continue
            pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk[ps:pe+1])
            if pam:
                wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
                pins[ref] = (fx + wx, fy + wy)
def seg_hits_other(x1, y1, x2, y2, allow_refs):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    for ref, cx, cy, rad in fp_boxes:
        if ref in allow_refs or not ref: continue
        if l2 == 0:
            d = math.hypot(cx - x1, cy - y1)
        else:
            t = max(0.0, min(1.0, ((cx - x1) * dx + (cy - y1) * dy) / l2))
            d = math.hypot(cx - (x1 + t * dx), cy - (y1 + t * dy))
        if d < rad:
            return True
    return False
# Remove prior auto-generated NRST segments if net_id matches or connect known NRST pins
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
    # Step 1: 0.15 mm exit past courtyard + ~2.5 mm
    d1 = min(3.2, dist * 0.25)
    p_a = (x1 + ux * d1, y1 + uy * d1)
    # Step 2: 0.30 mm after stepping out
    d2 = min(dist * 0.55, d1 + 2.5)
    p_b = (x1 + ux * d2, y1 + uy * d2)
    # Step 3: 0.40 mm in open space before tapering back near target pin
    d3 = max(d2 + 0.5, dist - min(3.2, dist * 0.25))
    p_c = (x1 + ux * d3, y1 + uy * d3)
    # Check if straight line crosses a non-endpoint footprint (e.g. C19); detour laterally if so
    if seg_hits_other(x1, y1, x2, y2, {r1, r2}):
        detour = 3.6
        for sign in (1.0, -1.0):
            ma = (p_a[0] + nx * detour * sign, p_a[1] + ny * detour * sign)
            mb = (p_b[0] + nx * detour * sign, p_b[1] + ny * detour * sign)
            mc = (p_c[0] + nx * detour * sign, p_c[1] + ny * detour * sign)
            pts = [(x1, y1), ma, mb, mc, (x2, y2)]
            ws = [0.1500, 0.3000, 0.4000, 0.1500]
            if not any(seg_hits_other(pts[k][0], pts[k][1], pts[k+1][0], pts[k+1][1], {r1, r2}) for k in range(4)):
                for k in range(4):
                    new_segs.append(f"  (segment (start {pts[k][0]:.4f} {pts[k][1]:.4f}) (end {pts[k+1][0]:.4f} {pts[k+1][1]:.4f}) (width {ws[k]:.4f}) (layer \"F.Cu\") (net {nrst_id}))")
                break
        else:
            pts = [(x1, y1), p_a, p_b, p_c, (x2, y2)]
            ws = [0.1500, 0.3000, 0.4000, 0.1500]
            for k in range(4):
                new_segs.append(f"  (segment (start {pts[k][0]:.4f} {pts[k][1]:.4f}) (end {pts[k+1][0]:.4f} {pts[k+1][1]:.4f}) (width {ws[k]:.4f}) (layer \"F.Cu\") (net {nrst_id}))")
    else:
        pts = [(x1, y1), p_a, p_b, p_c, (x2, y2)]
        ws = [0.1500, 0.3000, 0.4000, 0.1500]
        for k in range(4):
            new_segs.append(f"  (segment (start {pts[k][0]:.4f} {pts[k][1]:.4f}) (end {pts[k+1][0]:.4f} {pts[k+1][1]:.4f}) (width {ws[k]:.4f}) (layer \"F.Cu\") (net {nrst_id}))")
if new_segs:
    lp = txt.rfind(")")
    txt = txt[:lp] + "\n" + "\n".join(new_segs) + "\n)\n"
assert bal(txt), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"STEPPED_NRST_OK | pins={len(ordered_refs)} | segments_added={len(new_segs)} | balanced={bal(txt)}")
