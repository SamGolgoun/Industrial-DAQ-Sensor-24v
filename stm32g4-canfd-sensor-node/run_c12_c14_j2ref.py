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

er = find_end(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = find_end(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1

edits = []
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    nb = b
    if ref in ("C12", "C14"):
        am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", nb)
        if am:
            fx, fy = float(am.group(1)), float(am.group(2))
            fd = float(am.group(3)) if am.group(3) else 0.0
            ny = fy - 2.0 if ref == "C12" else fy + 2.2
            ds = int(fd) if fd.is_integer() else round(fd, 3)
            na = f"(at {fx:.3f} {ny:.3f} {ds})" if ds != 0 else f"(at {fx:.3f} {ny:.3f})"
            nb = nb[:am.start()] + na + nb[am.end():]
    elif ref == "J2":
        m = re.search(r"\(\s*property\s+\"Reference\"\s+\"J2\"", nb)
        if m:
            ps = m.start(); pe = find_end(nb, ps)
            if pe > ps:
                pb = nb[ps:pe+1]
                ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
                if ram:
                    rx, ry = float(ram.group(1)), float(ram.group(2))
                    rdeg = float(ram.group(3)) if ram.group(3) else 0.0
                    nry = -abs(ry) - 3.5 if abs(ry) > 0.1 else -3.8
                    rstr = f"(at {rx:.3f} {nry:.3f} {int(rdeg)})" if rdeg else f"(at {rx:.3f} {nry:.3f})"
                    nb = nb[:ps] + pb[:ram.start()] + rstr + pb[ram.end():] + nb[pe+1:]
    if nb != b: edits.append((st, en, nb))

for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True):
    txt = txt[:st] + nb + txt[en:]

assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print("C12_C14_J2REF_DONE | edits=", len(edits), "| balanced=", txt.count("(") == txt.count(")"))
