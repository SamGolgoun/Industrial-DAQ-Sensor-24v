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
def crt_top_y(blk):
    ys = []
    for m in re.finditer(r"\((?:fp_line|fp_rect)\b[^\n]*?\(start\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)\s*\(end\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)[^\n]*?(?:F|B)\.CrtYd", blk):
        ys.extend([float(m.group(1)), float(m.group(2))])
    if not ys:
        for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)[^\n]*?\(size\s+-?\d+(?:\.\d+)?\s+(-?\d+(?:\.\d+)?)\)", blk):
            py, hh = float(m.group(2)), float(m.group(4)) * 0.5
            ys.extend([py - hh, py + hh])
    return min(ys) if ys else -3.5
def move_ref_board_x(blk, ref_name, board_x=105.0, clearance=1.2):
    m = re.search(rf"\(\s*property\s+\"Reference\"\s+\"{re.escape(ref_name)}\"", blk)
    if not m: return blk, None
    ps = m.start(); pe = fe(blk, ps)
    if pe < 0: return blk, None
    pb = blk[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: return blk, None
    fpx, _ = get_fp_xy(blk)
    local_rx = round(board_x - fpx, 3)
    local_ry = round(crt_top_y(blk) - clearance, 3)
    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {local_rx:.3f} {local_ry:.3f} {rds})" if rds != 0 else f"(at {local_rx:.3f} {local_ry:.3f})"
    nb = pb[:ram.start()] + nstr + pb[ram.end():]
    return blk[:ps] + nb + blk[pe+1:], (fpx, local_rx, local_ry)
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
        nb, res = move_ref_board_x(b, ref, board_x=105.0, clearance=1.2)
        if res is not None:
            info[ref] = res
            if nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print("REF_X105_OK | info=", info, "| balanced=", txt.count("(") == txt.count(")"))
