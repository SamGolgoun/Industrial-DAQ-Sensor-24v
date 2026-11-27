import pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
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
assert txt.count("(") == txt.count(")"), "Unbalanced PCB"
end_r = find_end(txt, 0); i = 1; fps = []
while i < end_r:
    if txt[i] == "(":
        e = find_end(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
parsed = []
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", b)
    if am: parsed.append({"st": st, "en": en, "b": b, "ref": ref, "x": float(am.group(1)), "y": float(am.group(2)), "deg": float(am.group(3)) if am.group(3) else 0.0, "span": am.span()})
by_ref = {f["ref"]: f for f in parsed if f["ref"]}
dx, dy = -3.0, 4.0
if "J2" in by_ref: by_ref["J2"]["x"] = round(by_ref["J2"]["x"] + dx, 3)
if "J4" in by_ref:
    by_ref["J4"]["x"] = round(by_ref["J4"]["x"] + dx, 3)
    by_ref["J4"]["y"] = round(by_ref["J4"]["y"] + dy, 3)
    if "J2" in by_ref: by_ref["J4"]["x"] = by_ref["J2"]["x"]
edits = []
for f in parsed:
    if f["ref"] in {"J2", "J4"}:
        sa, ea = f["span"]; dv = float(f["deg"] % 360.0); ds = int(dv) if dv.is_integer() else round(dv, 3)
        na = f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f} {ds})" if ds != 0 else f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f})"
        edits.append((f["st"], f["en"], f["b"][:sa] + na + f["b"][ea:]))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
p.write_text(txt, encoding="utf-8")
print("ALIGN_OK | balanced=", txt.count("(") == txt.count(")"), "| J2=", by_ref.get("J2", {}).get("x"), by_ref.get("J2", {}).get("y"), "| J4=", by_ref.get("J4", {}).get("x"), by_ref.get("J4", {}).get("y"))