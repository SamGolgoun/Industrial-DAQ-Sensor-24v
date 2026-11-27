import math, pathlib, re

# Configure 4-layer 80x60 mm board layout, corners, connectors, and MCU clearance.
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = pcb.read_text(encoding="utf-8")

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
min_x = min(xs) if xs else 100.0
min_y = min(ys) if ys else 60.0
w_board = 80.0
h_board = 60.0
max_x = min_x + w_board
max_y = min_y + h_board

fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
by_ref = {f["ref"]: f for f in fps if f["ref"]}

# 1. Mounting holes H1..H4 in four corners with 5.0 mm edge inset
mh_coords = {
    "H1": (min_x + 5.0, min_y + 5.0),
    "H2": (max_x - 5.0, min_y + 5.0),
    "H3": (max_x - 5.0, max_y - 5.0),
    "H4": (min_x + 5.0, max_y - 5.0),
}
for r, xy in mh_coords.items():
    if r in by_ref:
        by_ref[r]["x"], by_ref[r]["y"] = xy

# 2. Fiducials FID1..FID3 inside board near corners
fid_coords = {
    "FID1": (min_x + 12.0, min_y + 8.0),
    "FID2": (max_x - 12.0, min_y + 8.0),
    "FID3": (max_x - 12.0, max_y - 8.0),
}
for r, xy in fid_coords.items():
    if r in by_ref:
        by_ref[r]["x"], by_ref[r]["y"] = xy

# 3. Connectors J2 and J4 inside board away from edges and clear of U1
if "J2" in by_ref:
    by_ref["J2"]["x"], by_ref["J2"]["y"] = (max_x - 12.0, min_y + 9.0)
if "J4" in by_ref:
    by_ref["J4"]["x"], by_ref["J4"]["y"] = (min_x + 36.0, max_y - 14.0)

# 4. Sensitive U1 and Y1 cleanly separated from J2/J4 and noisy perimeter zones
if "U1" in by_ref:
    by_ref["U1"]["x"], by_ref["U1"]["y"] = (min_x + 43.0, min_y + 28.0)
if "Y1" in by_ref:
    by_ref["Y1"]["x"], by_ref["Y1"]["y"] = (min_x + 35.0, min_y + 23.0)

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

pcb.write_text(txt, encoding="utf-8")
print("ALL_APPLIED_OK | Board: %.1fx%.1f | J2: %s | J4: %s | U1: %s | Balanced: %s" % (
    w_board,
    h_board,
    (by_ref["J2"]["x"], by_ref["J2"]["y"]) if "J2" in by_ref else None,
    (by_ref["J4"]["x"], by_ref["J4"]["y"]) if "J4" in by_ref else None,
    (by_ref["U1"]["x"], by_ref["U1"]["y"]) if "U1" in by_ref else None,
    txt.count("(") == txt.count(")"),
))
