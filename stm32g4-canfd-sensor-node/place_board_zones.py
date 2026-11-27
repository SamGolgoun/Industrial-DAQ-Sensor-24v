import math, pathlib, re

# Distribute footprints across 4 board zones with sensitive/noisy isolation.
PCB_PATH = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = PCB_PATH.read_text(encoding="utf-8")

def find_end(s, idx):
    d = 0
    q = False
    esc = False
    for i in range(idx, len(s)):
        c = s[i]
        if esc:
            esc = False
        elif c == "\\":
            esc = True
        elif c == "\"\"":
            q = not q
        elif not q:
            if c == "(":
                d += 1
            elif c == ")":
                d -= 1
                if d == 0:
                    return i
    return -1

def iter_fps(s):
    er = find_end(s, 0)
    i = 1
    while i < er:
        if s[i] == "(":
            e = find_end(s, i)
            if e < 0:
                break
            b = s[i:e+1]
            if b.startswith("(footprint "):
                yield i, e+1, b
            i = e + 1
        else:
            i += 1

def parse_fp(st, en, b):
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    fx = fy = fdeg = 0.0
    at_sp = None
    i = 1
    d = 1
    q = False
    esc = False
    while i < len(b):
        c = b[i]
        if esc:
            esc = False
        elif c == "\\":
            esc = True
        elif c == "\"\"":
            q = not q
        elif not q:
            if c == "(":
                if d == 1 and re.match(r"^\(\s*at\b", b[i:i+10]):
                    ae = find_end(b, i)
                    am = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", b[i:ae+1])
                    if am:
                        fx = float(am.group(1))
                        fy = float(am.group(2))
                        fdeg = float(am.group(3)) if am.group(3) else 0.0
                        at_sp = (i, ae+1)
                        break
                d += 1
            elif c == ")":
                d -= 1
        i += 1
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp}

xs = [float(m.group(1)) for m in re.finditer(r"\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
ys = [float(m.group(2)) for m in re.finditer(r"\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
x0 = min(xs) if xs else 100.0
y0 = min(ys) if ys else 60.0
w, h = 80.0, 60.0
x1, y1 = x0 + w, y0 + h

fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
by_ref = {f["ref"]: f for f in fps if f["ref"]}

# Fixed corner/fiducial coordinates inside 80x60 mm board
fixed_pos = {
    "H1": (x0 + 5.0, y0 + 5.0),
    "H2": (x1 - 5.0, y0 + 5.0),
    "H3": (x1 - 5.0, y1 - 5.0),
    "H4": (x0 + 5.0, y1 - 5.0),
    "FID1": (x0 + 13.5, y0 + 7.5),
    "FID2": (x1 - 13.5, y0 + 7.5),
    "FID3": (x1 - 13.5, y1 - 7.5),
    # Zone 2 & Zone 4 connectors safely inside board boundaries
    "J2": (x1 - 12.0, y0 + 12.0),
    "J4": (x0 + 38.0, y1 - 13.0),
    # Zone 3 sensitive core MCU and crystal
    "U1": (x0 + 44.0, y0 + 28.0),
    "Y1": (x0 + 36.5, y0 + 23.5),
}

for r, xy in fixed_pos.items():
    if r in by_ref:
        by_ref[r]["x"], by_ref[r]["y"] = xy

# Zone buckets for remaining components
# Zone 1: Top-Left (24V / power / protection / noisy passives)
# Zone 2: Top-Right (CAN transceivers U2..U5 & comm helpers)
# Zone 3: Center-Right (decoupling & sensitive analog near U1/Y1)
# Zone 4: Bottom / remaining passives spread in clean grids
z1, z2, z3, z4 = [], [], [], []
for f in fps:
    r = f["ref"]
    if not r or r in fixed_pos:
        continue
    if r in {"U2", "U3", "U4", "U5"} or r.startswith("J"):
        z2.append(f)
    elif r.startswith(("L", "FB", "Q", "D", "F")):
        z1.append(f)
    elif r.startswith("U"):
        z3.append(f)
    elif r.startswith(("C", "R")):
        # Split decoupling/passives across zones cleanly
        if len(z1) < 12 and r.startswith("C"):
            z1.append(f)
        elif len(z3) < 12:
            z3.append(f)
        else:
            z4.append(f)
    else:
        z4.append(f)

def grid_place(items, gx0, gy0, cols, dx, dy):
    for idx, item in enumerate(items):
        col = idx % cols
        row = idx // cols
        item["x"] = round(gx0 + col * dx, 3)
        item["y"] = round(gy0 + row * dy, 3)

grid_place(z1, x0 + 14.0, y0 + 14.0, cols=4, dx=4.2, dy=4.2)
grid_place(z2, x1 - 24.0, y0 + 18.0, cols=2, dx=5.5, dy=5.0)
grid_place(z3, x0 + 34.0, y0 + 34.0, cols=4, dx=4.0, dy=4.0)
grid_place(z4, x0 + 14.0, y1 - 22.0, cols=6, dx=4.2, dy=4.0)

# Enforce clearance between J2 and U1
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
if diff > 0:
    txt = txt.rstrip() + "\n" + (")" * diff) + "\n"

PCB_PATH.write_text(txt, encoding="utf-8")
print("PLACED_4_ZONES_OK | total_fps=%d | z1=%d z2=%d z3=%d z4=%d | balanced=%s" % (
    len(fps), len(z1), len(z2), len(z3), len(z4), txt.count("(") == txt.count(")")
))
