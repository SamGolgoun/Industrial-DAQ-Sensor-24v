import pathlib, re

pcb_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = pcb_path.read_text(encoding="utf-8")

def is_balanced(s: str) -> bool:
    depth = 0
    quoted = False
    escaped = False
    for ch in s:
        if quoted:
            if escaped: escaped = False
            elif ch == chr(92): escaped = True
            elif ch == chr(34): quoted = False
        else:
            if ch == chr(34): quoted = True
            elif ch == "(": depth += 1
            elif ch == ")":
                depth -= 1
                if depth < 0: return False
    return depth == 0 and not quoted

assert is_balanced(txt), "Unbalanced input PCB"

def find_end(s: str, idx: int) -> int:
    depth = q = e = 0
    for i in range(idx, len(s)):
        c = s[i]
        if e: e = 0
        elif c == chr(92): e = 1
        elif c == chr(34): q ^= 1
        elif not q:
            depth += (c == "(") - (c == ")")
            if depth == 0: return i
    return -1

def iter_fps(s: str):
    er = find_end(s, 0); i = 1
    while i < er:
        if s[i] == "(":
            e = find_end(s, i)
            if e < 0: break
            b = s[i:e+1]
            if b.startswith("(footprint "): yield i, e+1, b
            i = e + 1
        else: i += 1

def hide_value(blk: str):
    cnt = 0; out = []; pos = 0
    for m in re.finditer(r"\(\s*property\s+\"Value\"\s+\"[^\"]*\"", blk):
        ps = m.start(); pe = find_end(blk, ps)
        if pe < 0: continue
        pb = blk[ps:pe+1]
        if "(hide yes)" not in pb:
            pb = re.sub(r"\(\s*hide\s+no\s*\)", "(hide yes)", pb) if re.search(r"\(\s*hide\s+no\s*\)", pb) else pb[:-1] + " (hide yes))"
            cnt += 1
        out.append(blk[pos:ps] + pb); pos = pe + 1
    out.append(blk[pos:])
    return "".join(out), cnt

fps = []
for st, en, blk in iter_fps(txt):
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", blk)
    ref = rm.group(1).strip() if rm else ""
    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", blk)
    fps.append({"st": st, "en": en, "blk": blk, "ref": ref, "x": float(am.group(1)) if am else 0.0, "y": float(am.group(2)) if am else 0.0, "deg": float(am.group(3)) if (am and am.group(3)) else 0.0, "span": am.span() if am else None})

by_ref = {f["ref"]: f for f in fps if f["ref"]}
dx_back = -3.5; dy_j4_up = -2.5; moved = {}
if "J2" in by_ref:
    f2 = by_ref["J2"]; ox, oy = f2["x"], f2["y"]; f2["x"] = round(ox + dx_back, 3); moved["J2"] = ((ox, oy), (f2["x"], f2["y"]))
if "J4" in by_ref:
    f4 = by_ref["J4"]; ox, oy = f4["x"], f4["y"]
    f4["x"] = round(ox + dx_back, 3); f4["y"] = round(oy + dy_j4_up, 3)
    if "J2" in by_ref: f4["x"] = by_ref["J2"]["x"]
    moved["J4"] = ((ox, oy), (f4["x"], f4["y"]))

hidden_total = 0; edits = []
for f in fps:
    nb, hc = hide_value(f["blk"]); hidden_total += hc
    if f["ref"] in {"J2", "J4"} and f["span"]:
        orig_at = f["blk"][f["span"][0]:f["span"][1]]
        dv = float(f["deg"] % 360.0); ds = int(dv) if dv.is_integer() else round(dv, 3)
        na = f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f} {ds})" if ds != 0 else f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f})"
        nb = nb.replace(orig_at, na, 1)
    if nb != f["blk"]: edits.append((f["st"], f["en"], nb))

for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True):
    txt = txt[:st] + nb + txt[en:]

assert is_balanced(txt), "Unbalanced patched PCB"
pcb_path.write_text(txt, encoding="utf-8")
print(f"APPLY_OK | J2={moved.get(chr(74)+chr(50))} | J4={moved.get(chr(74)+chr(52))} | hidden={hidden_total} | balanced={is_balanced(pcb_path.read_text(encoding=chr(117)+chr(116)+chr(102)+chr(45)+chr(56)))}")
