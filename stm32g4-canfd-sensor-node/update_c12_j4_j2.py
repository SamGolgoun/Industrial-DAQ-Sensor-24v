import pathlib, re

p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
assert txt.count("(") == txt.count(")"), "Unbalanced PCB"

def find_end(s, idx):
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
                if d == 1 and re.match(r"^\(\s*at\b", blk[i:i+10]):
                    ae = find_end(blk, i)
                    m = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", blk[i:ae+1])
                    if m: return i, ae+1, float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else 0.0
                d += 1
            elif c == ")": d -= 1
        i += 1
    return None

def set_ref_local_y(blk, ref_name, new_ry):
    m = re.search(rf"\(\s*property\s+\"Reference\"\s+\"{re.escape(ref_name)}\"", blk)
    if not m: return blk, False
    ps = m.start(); pe = find_end(blk, ps)
    if pe < 0: return blk, False
    pb = blk[ps:pe+1]
    ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: return blk, False
    rx = float(ram.group(1)); rdeg = float(ram.group(3)) if ram.group(3) else 0.0
    rds = int(rdeg) if float(rdeg).is_integer() else round(float(rdeg), 3)
    nstr = f"(at {rx:.3f} {new_ry:.3f} {rds})" if rds != 0 else f"(at {rx:.3f} {new_ry:.3f})"
    nb = pb[:ram.start()] + nstr + pb[ram.end():]
    return blk[:ps] + nb + blk[pe+1:], True

er = find_end(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = find_end(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1

by_ref = {}
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    if ref: by_ref[ref] = (st, en, b, get_fp_at(b))

target_c12_y = None
if "C8" in by_ref and "C14" in by_ref and by_ref["C8"][3] and by_ref["C14"][3]:
    y8 = by_ref["C8"][3][3]; y14 = by_ref["C14"][3][3]
    target_c12_y = round(0.45 * y8 + 0.55 * y14, 3)

edits = []; log = []
for ref, (st, en, b, at_info) in by_ref.items():
    nb = b; changed = False
    if ref == "C12" and at_info:
        sa, ea, fx, fy, fd = at_info
        ny = target_c12_y if target_c12_y is not None else round(fy + 2.5, 3)
        ds = int(fd) if float(fd).is_integer() else round(float(fd), 3)
        na = f"(at {fx:.3f} {ny:.3f} {ds})" if ds != 0 else f"(at {fx:.3f} {ny:.3f})"
        nb = nb[:sa] + na + nb[ea:]; changed = True
        log.append(f"C12 Y: {fy:.3f} -> {ny:.3f}")
    elif ref == "J4":
        nb, ok = set_ref_local_y(nb, "J4", -3.6)
        if ok: changed = True; log.append("J4 Reference placed above footprint (-3.6)")
    elif ref == "J2":
        nb, ok = set_ref_local_y(nb, "J2", -1.6)
        if ok: changed = True; log.append("J2 Reference placed close to courtyard (-1.6)")
    if changed and nb != b: edits.append((st, en, nb))

for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True):
    txt = txt[:st] + nb + txt[en:]

assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print("UPDATE_OK | edits=", len(edits), "| balanced=", txt.count("(") == txt.count(")"))
for item in log: print(" ", item)
