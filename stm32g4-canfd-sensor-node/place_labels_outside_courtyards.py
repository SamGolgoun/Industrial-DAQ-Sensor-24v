import math, pathlib, re
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
def get_fp_xy_deg(blk):
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
def local_crtyd(blk):
    xs, ys = [], []
    for m in re.finditer(r"\((?:fp_line|fp_rect|fp_poly)\b[^\n]*?(?:F|B)\.CrtYd", blk):
        seg = m.group(0)
        for nm in re.finditer(r"\((?:start|end|xy)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", seg):
            xs.append(float(nm.group(1))); ys.append(float(nm.group(2)))
    if not xs:
        for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)[^\n]*?\(size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", blk):
            px, py, hw, hh = float(m.group(1)), float(m.group(2)), float(m.group(3))*0.5, float(m.group(4))*0.5
            xs.extend([px-hw, px+hw]); ys.extend([py-hh, py+hh])
    if not xs: xs, ys = [-0.9, 0.9], [-0.9, 0.9]
    return min(xs), min(ys), max(xs), max(ys)
def world_box(fx, fy, deg, lx0, ly0, lx1, ly1, margin=0.45):
    pts = [(lx0,ly0),(lx1,ly0),(lx1,ly1),(lx0,ly1)]
    wpts = [rot(px, py, deg) for px, py in pts]
    wx = [fx + p[0] for p in wpts]; wy = [fy + p[1] for p in wpts]
    return (min(wx)-margin, min(wy)-margin, max(wx)+margin, max(wy)+margin)
def tbox(wx, wy, w=2.2, h=1.0, pad=0.35):
    return (wx - w*0.5 - pad, wy - h*0.5 - pad, wx + w*0.5 + pad, wy + h*0.5 + pad)
def ov(a, b): return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
all_crtyd_boxes = []
for _, _, b in fps:
    fx, fy, fd = get_fp_xy_deg(b)
    lx0, ly0, lx1, ly1 = local_crtyd(b)
    all_crtyd_boxes.append(world_box(fx, fy, fd, lx0, ly0, lx1, ly1, margin=0.45))
placed_labels = []; moved_refs = []; edits = []
for st, en, b in fps:
    fx, fy, fd = get_fp_xy_deg(b)
    lx0, ly0, lx1, ly1 = local_crtyd(b)
    wpts = [rot(px, py, fd) for px, py in [(lx0,ly0),(lx1,ly0),(lx1,ly1),(lx0,ly1)]]
    cx0, cy0, cx1, cy1 = min(p[0] for p in wpts), min(p[1] for p in wpts), max(p[0] for p in wpts), max(p[1] for p in wpts)
    m = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]*)\"", b)
    if not m: continue
    ref = m.group(1).strip()
    ps = m.start(); pe = fe(b, ps)
    if pe < 0: continue
    pb = b[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: continue
    olx, oly = float(ram.group(1)), float(ram.group(2))
    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    cands = [
        (0.0, cy0 - 1.15),
        (0.0, cy1 + 1.15),
        (cx1 + 1.55, 0.0),
        (cx0 - 1.55, 0.0),
        (cx1 + 1.55, cy0 - 0.9),
        (cx0 - 1.55, cy0 - 0.9),
        (cx1 + 1.55, cy1 + 0.9),
        (cx0 - 1.55, cy1 + 0.9),
    ]
    for step_r in range(1, 16):
        d_off = 1.1 + step_r * 0.45
        for k in range(12):
            ang = 2.0 * math.pi * k / 12.0
            cands.append((d_off * math.cos(ang), d_off * math.sin(ang)))
    nlx, nly = olx, oly
    for clx, cly in cands:
        tb = tbox(fx + clx, fy + cly)
        if not any(ov(tb, cb) for cb in all_crtyd_boxes) and not any(ov(tb, lb) for lb in placed_labels):
            nlx, nly = round(clx, 3), round(cly, 3)
            break
    placed_labels.append(tbox(fx + nlx, fy + nly))
    if (round(nlx, 3), round(nly, 3)) != (round(olx, 3), round(oly, 3)):
        moved_refs.append(ref)
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {nlx:.3f} {nly:.3f} {rds})" if rds != 0 else f"(at {nlx:.3f} {nly:.3f})"
    nb = b[:ps] + pb[:ram.start()] + nstr + pb[ram.end():] + b[pe+1:]
    if nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"LBL_NO_CRTYD_OK | moved_count={len(moved_refs)} | moved={moved_refs[:25]} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
