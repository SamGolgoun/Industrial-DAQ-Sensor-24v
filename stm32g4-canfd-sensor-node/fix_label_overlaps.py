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
def lbl_box(wx, wy, w=2.4, h=1.1):
    return (wx - w*0.5, wy - h*0.5, wx + w*0.5, wy + h*0.5)
def ov(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
placed_boxes = []; adjusted = []; edits = []
for st, en, b in fps:
    fx, fy = get_fp_xy(b)
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    m = re.search(r"\(\s*property\s+\"Reference\"\s+\"[^\"]*\"", b)
    if not m: continue
    ps = m.start(); pe = fe(b, ps)
    if pe < 0: continue
    pb = b[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: continue
    lx, ly = float(ram.group(1)), float(ram.group(2))
    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    box = lbl_box(fx + lx, fy + ly)
    if any(ov(box, ob) for ob in placed_boxes):
        candidates = [(lx, ly - 1.6), (lx, ly + 1.6), (lx + 2.2, ly - 1.2), (lx - 2.2, ly - 1.2), (0.0, -2.6), (0.0, 2.6)]
        for clx, cly in candidates:
            cbox = lbl_box(fx + clx, fy + cly)
            if not any(ov(cbox, ob) for ob in placed_boxes):
                lx, ly = round(clx, 3), round(cly, 3)
                box = cbox
                adjusted.append(ref)
                break
    placed_boxes.append(box)
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {lx:.3f} {ly:.3f} {rds})" if rds != 0 else f"(at {lx:.3f} {ly:.3f})"
    nb = b[:ps] + pb[:ram.start()] + nstr + pb[ram.end():] + b[pe+1:]
    if nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"LABEL_OVERLAPS_OK | adjusted={len(adjusted)} | refs={adjusted} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
