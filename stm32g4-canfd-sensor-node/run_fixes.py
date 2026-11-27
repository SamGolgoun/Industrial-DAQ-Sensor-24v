import pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
assert txt.count("(") == txt.count(")"), "Unbalanced"
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
def get_at(b):
    i = 1; d = 1; q = e = 0
    while i < len(b):
        c = b[i]
        if e: e = 0
        elif c == chr(92): e = 1
        elif c == chr(34): q ^= 1
        elif not q:
            if c == "(":
                if d == 1 and re.match(r"^\(\s*at\b", b[i:i+10]):
                    ae = fe(b, i); m = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", b[i:ae+1])
                    if m: return i, ae+1, float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else 0.0
                d += 1
            elif c == ")": d -= 1
        i += 1
    return None
def set_ry(b, ref, ry_new):
    m = re.search(rf"\(\s*property\s+\"Reference\"\s+\"{re.escape(ref)}\"", b)
    if not m: return b
    ps = m.start(); pe = fe(b, ps)
    if pe < 0: return b
    pb = b[ps:pe+1]; ram = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", pb)
    if not ram: return b
    rx = float(ram.group(1)); rd = float(ram.group(3)) if ram.group(3) else 0.0
    rds = int(rd) if float(rd).is_integer() else round(float(rd), 3)
    ns = f"(at {rx:.3f} {ry_new:.3f} {rds})" if rds != 0 else f"(at {rx:.3f} {ry_new:.3f})"
    return b[:ps] + pb[:ram.start()] + ns + pb[ram.end():] + b[pe+1:]
def hide_val(b):
    out = []; pos = 0
    for m in re.finditer(r"\(\s*property\s+\"Value\"\s+\"[^\"]*\"", b):
        ps = m.start(); pe = fe(b, ps)
        if pe < 0: continue
        pb = b[ps:pe+1]
        if "(hide yes)" not in pb:
            pb = re.sub(r"\(\s*hide\s+no\s*\)", "(hide yes)", pb) if re.search(r"\(\s*hide\s+no\s*\)", pb) else pb[:-1] + " (hide yes))"
        out.append(b[pos:ps] + pb); pos = pe + 1
    out.append(b[pos:]); return "".join(out)
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
by_ref = {}
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    if ref: by_ref[ref] = (st, en, b, get_at(b))
mid_y = round(0.45*by_ref["C8"][3][3] + 0.55*by_ref["C14"][3][3], 3) if ("C8" in by_ref and "C14" in by_ref and by_ref["C8"][3] and by_ref["C14"][3]) else None
edits = []
for ref, (st, en, b, ai) in by_ref.items():
    nb = hide_val(b)
    if ref == "C12" and ai and mid_y is not None:
        sa, ea, fx, fy, fd = get_at(nb); ds = int(fd) if float(fd).is_integer() else round(float(fd), 3)
        na = f"(at {fx:.3f} {mid_y:.3f} {ds})" if ds != 0 else f"(at {fx:.3f} {mid_y:.3f})"
        nb = nb[:sa] + na + nb[ea:]
    elif ref == "J4": nb = set_ry(nb, "J4", -3.6)
    elif ref == "J2": nb = set_ry(nb, "J2", -1.6)
    if nb != b: edits.append((st, en, nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced out"
p.write_text(txt, encoding="utf-8")
print("RUN_FIXES_OK | edits=", len(edits), "| balanced=", txt.count("(") == txt.count(")"))
