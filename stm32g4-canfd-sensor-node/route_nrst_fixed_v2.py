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
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt):
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
    max_r = 2.2
    for pm in re.finditer(r"\(pad\b", blk):
        ps = pm.start(); pe = fe(blk, ps)
        if pe < 0: continue
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk[ps:pe+1])
        if pam:
            wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
            max_r = max(max_r, math.hypot(wx, wy) + 1.5)
    fp_boxes.append((ref, fx, fy, max_r))
    if ref in targets:
        for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"", blk):
            if pm.group(1).strip() != targets[ref]: continue
            ps = pm.start(); pe = fe(blk, ps)
            if pe < 0: continue
            pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk[ps:pe+1])
            if pam:
                wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
                pins[ref] = (fx + wx, fy + wy)
def pt_seg_d(cx, cy, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    if l2 == 0: return math.hypot(cx - x1, cy - y1)
    t = max(0.0, min(1.0, ((cx - x1) * dx + (cy - y1) * dy) / l2))
    return math.hypot(cx - (x1 + t * dx), cy - (y1 + t * dy))
def seg_hits(x1, y1, x2, y2, allow_refs):
    for ref, cx, cy, rad in fp_boxes:
        if ref in allow_refs or not ref: continue
        if pt_seg_d(cx, cy, x1, y1, x2, y2) < rad:
            return True
    return False
# Remove old segments that touch NRST pins or belong to NRST net
old_segs = list(re.finditer(r"\(segment\b", txt))
kept_spans = []
last_pos = 0
removed_count = 0
pin_coords = list(pins.values())
for sm in old_segs:
    ss = sm.start(); se = fe(txt, ss)
    if se < 0 or ss < last_pos: continue
    sblk = txt[ss:se+1]
    spts = [(float(a), float(b)) for a, b in re.findall(r"\((?:start|end)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", sblk)]
    snm = re.search(r"\(\s*net\s+(\d+)\s*\)", sblk)
    snid = int(snm.group(1)) if snm else -1
    near_nrst_pin = any(math.hypot(px - qx, py - qy) < 1.8 for px, py in spts for qx, qy in pin_coords)
    if (nrst_id > 0 and snid == nrst_id) or near_nrst_pin:
        kept_spans.append(txt[last_pos:ss])
        removed_count += 1
        last_pos = se + 1
kept_spans.append(txt[last_pos:])
txt = "".join(kept_spans)
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
    # Step out of dense pin at 0.1500 mm, then step up to 0.3000 mm and keep 0.3000 mm
    d_exit = min(2.0, dist * 0.2)
    p_exit = (x1 + ux * d_exit, y1 + uy * d_exit)
    # Wide 45-deg style fan-out and bypass around C19 / any obstacle
    best_pts = None
    for off in (4.5, 6.5, 8.5, -4.5, -6.5, -8.5):
        p_fan = (p_exit[0] + ux * 2.5 + nx * (off * 0.7), p_exit[1] + uy * 2.5 + ny * (off * 0.7))
        p_byp = (x1 + ux * (dist * 0.55) + nx * off, y1 + uy * (dist * 0.55) + ny * off)
        p_app = (x2 - ux * min(2.0, dist * 0.2) + nx * (off * 0.25), y2 - uy * min(2.0, dist * 0.2) + ny * (off * 0.25))
        cand = [(x1, y1), p_exit, p_fan, p_byp, p_app, (x2, y2)]
        if not any(seg_hits(cand[k][0], cand[k][1], cand[k+1][0], cand[k+1][1], {r1, r2}) for k in range(len(cand) - 1)):
            best_pts = cand
            break
    if best_pts is None:
        best_pts = [(x1, y1), p_exit, (x2, y2)]
    for k in range(len(best_pts) - 1):
        w = 0.1500 if k == 0 else 0.3000
        new_segs.append(f"  (segment (start {best_pts[k][0]:.4f} {best_pts[k][1]:.4f}) (end {best_pts[k+1][0]:.4f} {best_pts[k+1][1]:.4f}) (width {w:.4f}) (layer \"F.Cu\") (net {nrst_id}))")
if new_segs:
    lp = txt.rfind(")")
    txt = txt[:lp] + "\n" + "\n".join(new_segs) + "\n)\n"
assert bal(txt), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
tot_segs = len(re.findall(r"\(segment\b", txt))
print(f"FIXED_NRST_V2_OK | pins_found={ordered_refs} | removed={removed_count} | added={len(new_segs)} | total_segments={tot_segs} | balanced={bal(txt)}")
