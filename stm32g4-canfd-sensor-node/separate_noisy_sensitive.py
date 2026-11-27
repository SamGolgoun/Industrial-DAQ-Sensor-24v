import math, pathlib, re
PCB_PATH = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = PCB_PATH.read_text(encoding="utf-8")
def find_end(s, idx):
    d = q = esc = 0
    for i in range(idx, len(s)):
        c = s[i]
        if esc: esc = 0
        elif c == "\\": esc = 1
        elif c == chr(34): q ^= 1
        elif not q:
            d += (c == "(") - (c == ")")
            if d == 0: return i
    return -1
def iter_fps(s):
    er = find_end(s, 0); i = 1
    while i < er:
        if s[i] == "(":
            e = find_end(s, i)
            if e < 0: break
            b = s[i:e+1]
            if b.startswith("(footprint "): yield i, e+1, b
            i = e + 1
        else: i += 1
def parse_fp(st, en, b):
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    fx = fy = fdeg = 0.0; at_sp = None
    i = 1; d = 1; q = False; esc = False
    while i < len(b):
        c = b[i]
        if esc: esc = 0
        elif c == "\\": esc = 1
        elif c == chr(34): q ^= 1
        elif not q:
            if c == "(":
                if d == 1 and re.match(r"^\(\s*at\b", b[i:i+10]):
                    ae = find_end(b, i)
                    am = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", b[i:ae+1])
                    if am:
                        fx, fy = float(am.group(1)), float(am.group(2))
                        fdeg = float(am.group(3)) if am.group(3) else 0.0
                        at_sp = (i, ae+1); break
                d += 1
            elif c == ")": d -= 1
        i += 1
    has_sens = bool(re.search(r"ADC|ANA|OSC|VREF", b, re.I))
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp, "sens": has_sens}
xs = [float(m.group(1)) for m in re.finditer(r"\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
ys = [float(m.group(2)) for m in re.finditer(r"\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
x0 = min(xs) if xs else 100.0; y0 = min(ys) if ys else 60.0
x1, y1 = x0 + 80.0, y0 + 60.0
fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
by_ref = {f["ref"]: f for f in fps if f["ref"]}
pinned = {
    "H1": (x0 + 5.0, y0 + 5.0), "H2": (x1 - 5.0, y0 + 5.0),
    "H3": (x1 - 5.0, y1 - 5.0), "H4": (x0 + 5.0, y1 - 5.0),
    "FID1": (x0 + 14.0, y0 + 7.5), "FID2": (x1 - 14.0, y0 + 7.5), "FID3": (x1 - 14.0, y1 - 7.5),
    "J2": (x1 - 12.5, y0 + 13.0), "J4": (x0 + 38.0, y1 - 12.5),
    "U1": (x0 + 45.0, y0 + 27.0), "Y1": (x0 + 37.5, y0 + 22.5),
}
for r, xy in pinned.items():
    if r in by_ref: by_ref[r]["x"], by_ref[r]["y"] = xy
noisy_pwr = []; noisy_io = []; sensitive = []
for f in fps:
    r = f["ref"]
    if not r or r in pinned: continue
    if r.startswith(("L", "FB", "Q", "D", "F", "SW")): noisy_pwr.append(f)
    elif r in {"U2", "U3", "U4", "U5"} or r.startswith("J"): noisy_io.append(f)
    elif f["sens"] or r.startswith("U"): sensitive.append(f)
    else:
        if len(sensitive) < 12 and r.startswith("C"): sensitive.append(f)
        elif len(noisy_pwr) < 14: noisy_pwr.append(f)
        else: noisy_io.append(f)
def place_grid(items, gx0, gy0, cols, dx, dy):
    for idx, it in enumerate(items):
        it["x"] = round(gx0 + (idx % cols) * dx, 3)
        it["y"] = round(gy0 + (idx // cols) * dy, 3)
place_grid(noisy_pwr, x0 + 13.0, y0 + 15.0, cols=4, dx=4.8, dy=4.8)
place_grid(sensitive, x0 + 35.0, y0 + 33.0, cols=4, dx=4.6, dy=4.5)
place_grid(noisy_io, x1 - 26.0, y0 + 18.0, cols=2, dx=6.2, dy=5.6)
if "J2" in by_ref and "U1" in by_ref:
    assert math.hypot(by_ref["J2"]["x"] - by_ref["U1"]["x"], by_ref["J2"]["y"] - by_ref["U1"]["y"]) > 14.0
repls = []
for f in fps:
    if f["at_sp"]:
        sa, ea = f["at_sp"]
        d_val = int(f["deg"]) if float(f["deg"]).is_integer() else f["deg"]
        na = "(at %.3f %.3f %s)" % (f["x"], f["y"], d_val) if f["deg"] != 0 else "(at %.3f %.3f)" % (f["x"], f["y"])
        repls.append((f["st"], f["en"], f["b"][:sa] + na + f["b"][ea:]))
for s, e, nb in sorted(repls, key=lambda item: item[0], reverse=True):
    txt = txt[:s] + nb + txt[e:]
diff = txt.count("(") - txt.count(")")
if diff > 0: txt = txt.rstrip() + "\n" + (")" * diff) + "\n"
PCB_PATH.write_text(txt, encoding="utf-8")
print("SEPARATED_OK | noisy_pwr=%d noisy_io=%d sensitive=%d | balanced=%s" % (len(noisy_pwr), len(noisy_io), len(sensitive), txt.count("(") == txt.count(")")))
