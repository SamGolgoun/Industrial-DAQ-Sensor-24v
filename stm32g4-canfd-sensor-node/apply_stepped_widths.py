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
net_names = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt):
    net_names[int(m.group(1))] = m.group(2).strip()
def rot(lx, ly, deg):
    r = math.radians(deg)
    return lx * math.cos(r) - ly * math.sin(r), lx * math.sin(r) + ly * math.cos(r)
other_pts = []
for m in re.finditer(r"\(footprint\b", txt):
    fs = m.start(); fe_i = fe(txt, fs)
    if fe_i < 0: continue
    blk = txt[fs:fe_i+1]
    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", blk)
    fx = float(am.group(1)) if am else 0.0
    fy = float(am.group(2)) if am else 0.0
    fd = float(am.group(3)) if (am and am.group(3)) else 0.0
    for pm in re.finditer(r"\(pad\b", blk):
        ps = pm.start(); pe = fe(blk, ps)
        if pe < 0: continue
        pblk = blk[ps:pe+1]
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pblk)
        pnm = re.search(r"\(\s*net\s+(\d+)\b", pblk)
        if pam:
            wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
            other_pts.append((fx + wx, fy + wy, int(pnm.group(1)) if pnm else 0))
def pt_seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    if l2 == 0: return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))
seg_matches = list(re.finditer(r"\(segment\b[^\n]*\)", txt))
parsed_segs = []
for sm in seg_matches:
    sb = sm.group(0)
    pts = [(float(a), float(b)) for a, b in re.findall(r"\((?:start|end)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", sb)]
    wm = re.search(r"\(\s*width\s+(-?\d+(?:\.\d+)?)\s*\)", sb)
    nm = re.search(r"\(\s*net\s+(\d+)\s*\)", sb)
    parsed_segs.append((sm.start(), sm.end(), sb, pts, float(wm.group(1)) if wm else 0.15, int(nm.group(1)) if nm else 0))
min_clr = 0.15
stepped = 0; kept_tight = 0
replacements = []
for idx, (st, en, sb, pts, cur_w, nid) in enumerate(parsed_segs):
    if len(pts) < 2: continue
    (x1, y1), (x2, y2) = pts[0], pts[-1]
    nname = net_names.get(nid, "").upper()
    is_pwr = any(k in nname for k in ("GND", "3V3", "VCC", "VDD", "VBUS", "PWR"))
    ladder = [0.50, 0.40, 0.30, 0.25, 0.20, 0.15] if is_pwr else [0.40, 0.30, 0.25, 0.20, 0.15]
    cands = [w for w in ladder if w >= cur_w - 1e-6]
    if cur_w not in cands: cands.append(cur_w)
    cands = sorted(set(cands), reverse=True)
    chosen_w = cur_w
    for cand_w in cands:
        req_gap = cand_w * 0.5 + min_clr
        ok = True
        for ox, oy, onid in other_pts:
            if onid != nid and pt_seg_dist(ox, oy, x1, y1, x2, y2) < req_gap + 0.15:
                ok = False; break
        if not ok: continue
        for jdx, (_, _, _, opts, ow, onid) in enumerate(parsed_segs):
            if jdx == idx or len(opts) < 2 or onid == nid: continue
            (ox1, oy1), (ox2, oy2) = opts[0], opts[-1]
            d_ends = min(pt_seg_dist(ox1, oy1, x1, y1, x2, y2), pt_seg_dist(ox2, oy2, x1, y1, x2, y2), pt_seg_dist(x1, y1, ox1, oy1, ox2, oy2), pt_seg_dist(x2, y2, ox1, oy1, ox2, oy2))
            if d_ends < (cand_w * 0.5 + ow * 0.5 + min_clr):
                ok = False; break
        if ok:
            chosen_w = cand_w; break
    if abs(chosen_w - cur_w) > 1e-6:
        nb = re.sub(r"\(\s*width\s+-?\d+(?:\.\d+)?\s*\)", f"(width {chosen_w:.4f})", sb, count=1)
        replacements.append((st, en, nb))
        stepped += 1
    else:
        kept_tight += 1
for st, en, nb in sorted(replacements, key=lambda x: x[0], reverse=True):
    txt = txt[:st] + nb + txt[en:]
assert bal(txt), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"STEP_ALL_TRACES_OK | total_segs={len(parsed_segs)} | stepped_wider={stepped} | kept_as_is={kept_tight} | balanced={bal(txt)}")
