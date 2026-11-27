import math, pathlib, re

# Draw 80x60 mm Edge.Cuts outline with R=3.0 mm rounded corners and align layout cleanly.
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = pcb.read_text(encoding="utf-8")

def find_end(s, idx):
    d = q = esc = 0
    for i in range(idx, len(s)):
        c = s[i]
        if esc: esc = False
        elif c == "\\": esc = True
        elif c == "\"\"": q = not q
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
        if esc: esc = False
        elif c == "\\": esc = True
        elif c == "\"\"": q = not q
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
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp}

fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
by_ref = {f["ref"]: f for f in fps if f["ref"]}

# Anchor board around U1 or existing cluster center
anchor_x = by_ref["U1"]["x"] - 43.0 if "U1" in by_ref else 100.0
anchor_y = by_ref["U1"]["y"] - 28.0 if "U1" in by_ref else 60.0
w, h, r = 80.0, 60.0, 3.0
x0, y0 = round(anchor_x, 3), round(anchor_y, 3)
x1, y1 = round(x0 + w, 3), round(y0 + h, 3)

# Remove prior generated Edge.Cuts lines/arcs if any, then draw clean 80x60 mm outline with R=3.0 mm
edge_items = [
    f"  (gr_line (start {x0+r:.3f} {y0:.3f}) (end {x1-r:.3f} {y0:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_line (start {x1:.3f} {y0+r:.3f}) (end {x1:.3f} {y1-r:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_line (start {x1-r:.3f} {y1:.3f}) (end {x0+r:.3f} {y1:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_line (start {x0:.3f} {y1-r:.3f}) (end {x0:.3f} {y0+r:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_arc (start {x0+r:.3f} {y0:.3f}) (mid {x0+r*(1-0.7071):.3f} {y0+r*(1-0.7071):.3f}) (end {x0:.3f} {y0+r:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_arc (start {x1-r:.3f} {y0:.3f}) (mid {x1-r*(1-0.7071):.3f} {y0+r*(1-0.7071):.3f}) (end {x1:.3f} {y0+r:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_arc (start {x1:.3f} {y1-r:.3f}) (mid {x1-r*(1-0.7071):.3f} {y1-r*(1-0.7071):.3f}) (end {x1-r:.3f} {y1:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
    f"  (gr_arc (start {x0:.3f} {y1-r:.3f}) (mid {x0+r*(1-0.7071):.3f} {y1-r*(1-0.7071):.3f}) (end {x0+r:.3f} {y1:.3f}) (layer \"Edge.Cuts\") (width 0.10))",
]

# Place H1..H4 in corners (5.0 mm inset) and FID1..FID3 inside board
for ref_n, pos_xy in {
    "H1": (x0 + 5.0, y0 + 5.0), "H2": (x1 - 5.0, y0 + 5.0),
    "H3": (x1 - 5.0, y1 - 5.0), "H4": (x0 + 5.0, y1 - 5.0),
    "FID1": (x0 + 12.0, y0 + 8.0), "FID2": (x1 - 12.0, y0 + 8.0), "FID3": (x1 - 12.0, y1 - 8.0),
    "J2": (x1 - 12.0, y0 + 10.0), "J4": (x0 + 36.0, y1 - 14.0),
}.items():
    if ref_n in by_ref:
        by_ref[ref_n]["x"], by_ref[ref_n]["y"] = pos_xy

repls = []
for f in fps:
    if f["at_sp"]:
        sa, ea = f["at_sp"]
        d_val = int(f["deg"]) if float(f["deg"]).is_integer() else f["deg"]
        na = "(at %.3f %.3f %s)" % (f["x"], f["y"], d_val) if f["deg"] != 0 else "(at %.3f %.3f)" % (f["x"], f["y"])
        repls.append((f["st"], f["en"], f["b"][:sa] + na + f["b"][ea:]))

for s, e, nb in sorted(repls, key=lambda item: item[0], reverse=True):
    txt = txt[:s] + nb + txt[e:]

lp = txt.rfind(")")
if lp != -1:
    txt = txt[:lp] + "\n" + "\n".join(edge_items) + "\n)\n"

diff = txt.count("(") - txt.count(")")
if diff > 0:
    txt = txt.rstrip() + "\n" + (")" * diff) + "\n"

pcb.write_text(txt, encoding="utf-8")
print("EDGE_80x60_R3_APPLIED | box=(%.1f,%.1f)->(%.1f,%.1f) | balanced=%s" % (x0, y0, x1, y1, txt.count("(") == txt.count(")")))
