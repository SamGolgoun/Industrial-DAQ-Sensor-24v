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
def crt_top_y(blk):
    ys = []
    for m in re.finditer(r"\((?:fp_line|fp_rect)\b[^\n]*?\(start\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)\s*\(end\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)[^\n]*?(?:F|B)\.CrtYd", blk):
        ys.extend([float(m.group(1)), float(m.group(2))])
    if not ys:
        for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)[^\n]*?\(size\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)", blk):
            py, hh = float(m.group(2)), float(m.group(4)) * 0.5
            ys.extend([py - hh, py + hh])
    return min(ys) if ys else -4.0
def place_ref_above_courtyard(blk, ref_name, clearance=0.9):
    m = re.search(rf"\(\s*property\s+\"Reference\"\s+\"{re.escape(ref_name)}\"", blk)
    if not m: return blk, None
    ps = m.start(); pe = fe(blk, ps)
    if pe < 0: return blk, None
    pb = blk[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: return blk, None
    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    top_y = crt_top_y(blk)
    target_x = 0.0
    target_y = round(top_y - clearance, 3)
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {target_x:.3f} {target_y:.3f} {rds})" if rds != 0 else f"(at {target_x:.3f} {target_y:.3f})"
    nb = pb[:ram.start()] + nstr + pb[ram.end():]
    return blk[:ps] + nb + blk[pe+1:], (target_x, target_y, top_y)
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
edits = []; info = {}
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    if ref in ("J2", "J4"):
        nb, pos_info = place_ref_above_courtyard(b, ref, clearance=0.9)
        if pos_info is not None:
            info[ref] = pos_info
            if nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print("J2_J4_ABOVE_CRT_OK | info=", info, "| balanced=", txt.count("(") == txt.count(")"))
