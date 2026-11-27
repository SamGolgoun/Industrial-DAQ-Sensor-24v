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
def move_ref(blk, ref_name, new_rx, new_ry):
    m = re.search(rf"\(\s*property\s+\"Reference\"\s+\"{re.escape(ref_name)}\"", blk)
    if not m: return blk, False
    ps = m.start(); pe = fe(blk, ps)
    if pe < 0: return blk, False
    pb = blk[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: return blk, False
    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {new_rx:.3f} {new_ry:.3f} {rds})" if rds != 0 else f"(at {new_rx:.3f} {new_ry:.3f})"
    nb = pb[:ram.start()] + nstr + pb[ram.end():]
    return blk[:ps] + nb + blk[pe+1:], True
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
edits = []; moved = []
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    nb = b; ok = False
    if ref == "J2":
        nb, ok = move_ref(b, "J2", -2.2, -1.2)
        if ok: moved.append("J2 -> (-2.2, -1.2)")
    elif ref == "J4":
        nb, ok = move_ref(b, "J4", 0.0, -4.2)
        if ok: moved.append("J4 -> (0.0, -4.2)")
    if ok and nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print("J2_J4_ANN_OK | updated=", len(edits), "| balanced=", txt.count("(") == txt.count(")"))
for m_item in moved: print(" ", m_item)
