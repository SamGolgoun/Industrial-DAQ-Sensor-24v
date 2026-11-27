import pathlib, re
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
def get_fp_at(blk):
    i = 1; d = 1; q = e = 0
    while i < len(blk):
        c = blk[i]
        if e: e = 0
        elif c == chr(92): e = 1
        elif c == chr(34): q ^= 1
        elif not q:
            if c == "(":
                if d == 1 and re.match(r"^\(\s*at\b", b_seg if False else blk[i:i+10]):
                    ae = fe(blk, i)
                    m = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", blk[i:ae+1])
                    if m: return i, ae+1, float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else 0.0
                d += 1
            elif c == ")": d -= 1
        i += 1
    return None
def crt_top_y(blk):
    ys = []
    for m in re.finditer(r"\((?:fp_line|fp_rect)\b[^\n]*?\(start\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)\s*\(end\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)[^\n]*?(?:F|B)\.CrtYd", blk):
        ys.extend([float(m.group(1)), float(m.group(2))])
    if not ys:
        for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)[^\n]*?\(size\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)", blk):
            py, hh = float(m.group(2)), float(m.group(4)) * 0.5
            ys.extend([py - hh, py + hh])
    return min(ys) if ys else -3.5
def move_ref_above_crt(blk, ref_name, clearance=1.2):
    m = re.search(rf"\(\s*property\s+\"Reference\"\s+\"{re.escape(ref_name)}\"", blk)
    if not m: return blk
    ps = m.start(); pe = fe(blk, ps)
    if pe < 0: return blk
    pb = blk[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: return blk
    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    ny = round(crt_top_y(blk) - clearance, 3)
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at 0.000 {ny:.3f} {rds})" if rds != 0 else f"(at 0.000 {ny:.3f})"
    return blk[:ps] + pb[:ram.start()] + nstr + pb[ram.end():] + blk[pe+1:]
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
targets = {"J2": (105.0, 100.0), "J4": (105.0, 107.0)}
edits = []; done_refs = []
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    if ref in targets:
        tx, ty = targets[ref]
        at_info = get_fp_at(b)
        nb = b
        if at_info:
            sa, ea, _, _, fd = at_info
            ds = int(fd) if float(fd).is_integer() else round(float(fd), 3)
            na = f"(at {tx:.3f} {ty:.3f} {ds})" if ds != 0 else f"(at {tx:.3f} {ty:.3f})"
            nb = nb[:sa] + na + nb[ea:]
        nb = move_ref_above_crt(nb, ref, clearance=1.2)
        if nb != b: edits.append((st, en, nb))
        done_refs.append((ref, tx, ty))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print("COORDS_SET_OK | refs=", done_refs, "| balanced=", txt.count("(") == txt.count(")"))
