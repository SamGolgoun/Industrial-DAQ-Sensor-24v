import pathlib, re, uuid
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
assert txt.count("(") == txt.count(")"), "Unbalanced input PCB"
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
er = fe(txt, 0); i = 1; replaced = False
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint ") and re.search(r"\(\s*property\s+\"Reference\"\s+\"Q1\"", b):
            am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", b)
            x = float(am.group(1)) if am else 120.0; y = float(am.group(2)) if am else 80.0; deg = float(am.group(3)) if (am and am.group(3)) else 0.0
            vm = re.search(r"\(\s*property\s+\"Value\"\s+\"([^\"]*)\"", b)
            val = vm.group(1) if vm else "Q1"
            ds = int(deg) if deg.is_integer() else round(deg, 3)
            at_s = f"{x:.3f} {y:.3f} {ds}" if ds != 0 else f"{x:.3f} {y:.3f}"
            nb = (
                f"  (footprint \"Diode_SMD:D_SOD-123\" (layer \"F.Cu\") (uuid \"{uuid.uuid4()}\")\n"
                f"    (at {at_s})\n"
                f"    (descr \"Diode SMD SOD-123 2-pin\")\n"
                f"    (property \"Reference\" \"Q1\" (at 0.000 -1.900 0) (layer \"F.SilkS\") (uuid \"{uuid.uuid4()}\") (effects (font (size 0.8000 0.8000) (thickness 0.1200))))\n"
                f"    (property \"Value\" \"{val}\" (at 0.000 1.900 0) (layer \"F.Fab\") (hide yes) (uuid \"{uuid.uuid4()}\") (effects (font (size 0.8000 0.8000) (thickness 0.1200))))\n"
                f"    (fp_rect (start -1.4000 -0.9500) (end 1.4000 0.9500) (stroke (width 0.1200) (type solid)) (fill none) (layer \"F.CrtYd\") (uuid \"{uuid.uuid4()}\"))\n"
                f"    (pad \"1\" smd rect (at -1.6500 0.0000) (size 0.9000 1.0000) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\") (uuid \"{uuid.uuid4()}\"))\n"
                f"    (pad \"2\" smd rect (at 1.6500 0.0000) (size 0.9000 1.0000) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\") (uuid \"{uuid.uuid4()}\"))\n"
                f"  )"
            )
            txt = txt[:i] + nb + txt[e+1:]
            replaced = True; break
        i = e + 1
    else: i += 1
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"Q1_SOD123_OK | replaced={replaced} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
