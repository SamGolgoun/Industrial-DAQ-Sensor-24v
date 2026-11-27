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
def get_fp_xy(blk):
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
                    if m: return float(m.group(1)), float(m.group(2))
                d += 1
            elif c == ")": d -= 1
        i += 1
    return 0.0, 0.0
def local_body_box(blk, margin=0.4):
    xs, ys = [], []
    for m in re.finditer(r"\((?:fp_line|fp_rect)\b[^\n]*?\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)\s*\(end\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*?(?:F|B)\.CrtYd", blk):
        xs.extend([float(m.group(1)), float(m.group(3))]); ys.extend([float(m.group(2)), float(m.group(4))])
    if not xs:
        for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)[^\n]*?\(size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", blk):
            px, py, hw, hh = float(m.group(1)), float(m.group(2)), float(m.group(3))*0.5, float(m.group(4))*0.5
            xs.extend([px-hw, px+hw]); ys.extend([py-hh, py+hh])
    if not xs: xs, ys = [-1.0, 1.0], [-1.0, 1.0]
    return (min(xs)-margin, min(ys)-margin, max(xs)+margin, max(ys)+margin)
def tbox(wx, wy, w=2.2, h=1.0):
    return (wx - w*0.5, wy - h*0.5, wx + w*0.5, wy + h*0.5)
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
used_boxes = []; moved_refs = []; edits = []
for st, en, b in fps:
    fx, fy = get_fp_xy(b)
    lx0, lx1, ly0, ly1 = local_body_box(b, margin=0.35)
    body_w = (lx0 + fx, ly0 + fy, lx1 + fx, ly1 + fy)
    used_boxes.append(body_w)
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
        (lx1 + 1.1, (ly0 + ly1) * 0.5),
        (lx0 - 1.1, (ly0 + ly1) * 0.5),
        ((lx0 + lx1) * 0.5, ly0 - 0.8),
        ((lx0 + lx1) * 0.5, ly1 + 0.8),
        (lx1 + 1.4, ly0 - 0.6),
        (lx0 - 1.4, ly0 - 0.6),
        (lx1 + 1.4, ly1 + 0.6),
        (lx0 - 1.4, ly1 + 0.6),
    ]
    nlx, nly = olx, oly
    for clx, cly in cands:
        tb = tbox(fx + clx, fy + cly)
        if not any(ov(tb, ub) for ub in used_boxes):
            nlx, nly = round(clx, 3), round(cly, 3)
            break
    used_boxes.append(tbox(fx + nlx, fy + nly))
    if (round(nlx, 3), round(nly, 3)) != (round(olx, 3), round(oly, 3)):
        moved_refs.append(ref)
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {nlx:.3f} {nly:.3f} {rds})" if rds != 0 else f"(at {nlx:.3f} {nly:.3f})"
    nb = b[:ps] + pb[:ram.start()] + nstr + pb[ram.end():] + b[pe+1:]
    if nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"REF_BESIDE_OK | moved_count={len(moved_refs)} | moved={moved_refs[:20]} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
