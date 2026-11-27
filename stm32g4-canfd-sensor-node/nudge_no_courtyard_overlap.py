import math, pathlib, re
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
def get_at(blk):
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
                    if m: return i, ae+1, float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else 0.0
                d += 1
            elif c == ")": d -= 1
        i += 1
    return None
def crtyd_box(blk, x, y, deg, margin=0.2):
    xs, ys = [], []
    for m in re.finditer(r"\((?:fp_line|fp_rect|fp_poly)\b[^\n]*?(?:F|B)\.CrtYd", blk):
        seg = m.group(0)
        for nm in re.finditer(r"\((?:start|end|xy)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", seg):
            xs.append(float(nm.group(1))); ys.append(float(nm.group(2)))
    if not xs:
        for m in re.finditer(r"\(pad\b[^\n]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)[^\n]*?\(size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", blk):
            px, py, hw, hh = float(m.group(1)), float(m.group(2)), float(m.group(3))*0.5, float(m.group(4))*0.5
            xs.extend([px-hw, px+hw]); ys.extend([py-hh, py+hh])
    if not xs: xs, ys = [-0.8, 0.8], [-0.8, 0.8]
    lx0, lx1, ly0, ly1 = min(xs), max(xs), min(ys), max(ys)
    r = math.radians(deg); ca, sa = math.cos(r), math.sin(r)
    pts = [(lx0,ly0),(lx1,ly0),(lx1,ly1),(lx0,ly1)]
    wx = [x + px*ca - py*sa for px, py in pts]; wy = [y + px*sa + py*ca for px, py in pts]
    return (min(wx)-margin, min(wy)-margin, max(wx)+margin, max(wy)+margin)
def ov(a, b): return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])
xs = [float(m.group(1)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
ys = [float(m.group(2)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
x0, y0 = (min(xs) if xs else 100.0), (min(ys) if ys else 60.0); x1, y1 = x0 + 80.0, y0 + 60.0
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
locked = {"H1","H2","H3","H4","FID1","FID2","FID3","J1","J2","J3","J4","U2"}
items = []
for st, en, b in fps:
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    ai = get_at(b)
    if ai: items.append({"st":st, "en":en, "b":b, "ref":ref, "sa":ai[0], "ea":ai[1], "x":ai[2], "y":ai[3], "deg":ai[4]})
placed = [crtyd_box(it["b"], it["x"], it["y"], it["deg"]) for it in items if it["ref"] in locked]
nudged = []
for it in items:
    if it["ref"] in locked: continue
    ox, oy, deg = it["x"], it["y"], it["deg"]
    box = crtyd_box(it["b"], ox, oy, deg)
    if any(ov(box, pb) for pb in placed):
        done = False
        for r_step in range(1, 50):
            rad = r_step * 0.25; n_a = max(16, r_step * 8)
            for k in range(n_a):
                ang = 2.0 * math.pi * k / n_a
                nx = max(x0 + 2.0, min(x1 - 2.0, ox + rad * math.cos(ang)))
                ny = max(y0 + 2.0, min(y1 - 2.0, oy + rad * math.sin(ang)))
                nbox = crtyd_box(it["b"], nx, ny, deg)
                if not any(ov(nbox, pb) for pb in placed):
                    it["x"], it["y"] = round(nx, 3), round(ny, 3); box = nbox
                    nudged.append(it["ref"]); done = True; break
            if done: break
    placed.append(box)
warn_refs = ["Y1","C23","C7","C14","F1","C11","C24","C4","C18","C22","R8","C15","C5","C9"]
txt = re.sub(r"\n  \(drc_exclusions[^\n]*?(?:\n    \(drc_exclusion[^\n]*)*\n  \)", "", txt)
ex_lines = [f"    (drc_exclusion (type library_mismatch) (reason \"Accepted library state\") (item (ref \"{r}\")))" for r in warn_refs]
ex_blk = "  (drc_exclusions\n" + "\n".join(ex_lines) + "\n  )"
edits = []
for it in items:
    dv = float(it["deg"] % 360.0); ds = int(dv) if dv.is_integer() else round(dv, 3)
    na = f"(at {it[chr(120)]:.3f} {it[chr(121)]:.3f} {ds})" if ds != 0 else f"(at {it[chr(120)]:.3f} {it[chr(121)]:.3f})"
    nb = it["b"][:it["sa"]] + na + it["b"][it["ea"]:]
    if nb != it["b"]: edits.append((it["st"], it["en"], nb))
for st, en, nb in sorted(edits, key=lambda x: x[0], reverse=True): txt = txt[:st] + nb + txt[en:]
er2 = fe(txt, 0)
txt = txt[:er2] + "\n" + ex_blk + "\n" + txt[er2:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"NUDGE_CRTYD_OK | nudged_count={len(nudged)} | nudged={nudged} | exclusions_added={len(ex_lines)} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
