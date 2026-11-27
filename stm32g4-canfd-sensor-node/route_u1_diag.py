import math, pathlib, re

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
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b) or re.search(r"\(\s*fp_text\s+reference\s+\"?([^\"\s\)]+)", b)
    ref = rm.group(1).strip() if rm else ""
    fx = fy = fdeg = 0.0
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
                        break
                d += 1
            elif c == ")":
                d -= 1
        i += 1
    pads = []
    pi = 1
    while pi < len(b):
        if b[pi:pi+5] == "(pad ":
            pe = find_end(b, pi)
            if pe < 0:
                break
            pb = b[pi:pe+1]
            pm = re.match(r"^\(\s*pad\s+\"([^\"]*)\"", pb)
            pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pb)
            psm = re.search(r"\(\s*size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pb)
            pnm = re.search(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"", pb)
            if pm and pam and psm:
                pads.append({
                    "num": pm.group(1),
                    "lx": float(pam.group(1)),
                    "ly": float(pam.group(2)),
                    "sw": float(psm.group(1)),
                    "sh": float(psm.group(2)),
                    "code": int(pnm.group(1)) if pnm else 0,
                    "net": pnm.group(2) if pnm else "",
                })
            pi = pe + 1
        else:
            pi += 1
    return {"ref": ref, "x": fx, "y": fy, "deg": fdeg, "pads": pads}

def pxy(fp, pad):
    rad = math.radians(-fp["deg"])
    return fp["x"] + pad["lx"]*math.cos(rad) - pad["ly"]*math.sin(rad), fp["y"] + pad["lx"]*math.sin(rad) + pad["ly"]*math.cos(rad)

def pt_seg(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx*dx + dy*dy
    if l2 == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / l2))
    return math.hypot(px - (x1 + t*dx), py - (y1 + t*dy))

fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
by_ref = {f["ref"]: f for f in fps if f["ref"]}
u1 = by_ref.get("U1")
print("Total parsed footprints:", len(fps), "U1 found:", bool(u1))
if not u1:
    raise SystemExit(0)

all_pads = []
for fp in fps:
    for p in fp["pads"]:
        wx, wy = pxy(fp, p)
        all_pads.append({
            "ref": fp["ref"],
            "num": p["num"],
            "x": wx,
            "y": wy,
            "r": max(p["sw"], p["sh"])*0.5,
            "net": p["net"],
            "code": p["code"],
        })

u1_pins = [p for p in all_pads if p["ref"] == "U1" and p["net"] and p["net"] != "GND"]
print("U1 non-GND pads:", len(u1_pins))

segs = []
def clear_seg(x1, y1, x2, y2, w, net, clr):
    for pd in all_pads:
        if pd["net"] != net and pt_seg(pd["x"], pd["y"], x1, y1, x2, y2) < (pd["r"] + w*0.5 + clr):
            return False
    return True

routed = []
for up in u1_pins:
    tgs = [p for p in all_pads if p["net"] == up["net"] and not (p["ref"] == "U1" and p["num"] == up["num"])]
    if not tgs:
        continue
    near = min(tgs, key=lambda tp: math.hypot(tp["x"] - up["x"], tp["y"] - up["y"]))
    x1, y1, x2, y2 = up["x"], up["y"], near["x"], near["y"]
    if math.hypot(x2 - x1, y2 - y1) < 0.2:
        continue
    vx = x1 - u1["x"]
    vy = y1 - u1["y"]
    if abs(vx) >= abs(vy):
        edx = 1.0 if vx >= 0 else -1.0
        edy = 0.0
    else:
        edx = 0.0
        edy = 1.0 if vy >= 0 else -1.0
    nx1 = round(x1 + edx*0.65, 4)
    ny1 = round(y1 + edy*0.65, 4)
    wide = near["ref"].startswith(("R", "D", "L", "FB", "F")) or ("3V3" in up["net"])
    tw = 0.35 if wide else 0.25
    ew = 0.40 if (wide and near["r"] >= 0.35) else 0.25
    dx = x2 - nx1
    dy = y2 - ny1
    diag = min(abs(dx), abs(dy))
    sx = 1.0 if dx >= 0 else -1.0
    sy = 1.0 if dy >= 0 else -1.0
    mx1 = round(nx1 + sx*diag, 4)
    my1 = round(ny1 + sy*diag, 4)
    cand = [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, mx1, my1, tw), (mx1, my1, x2, y2, ew)]
    cc = [(a, b, e, f, w) for a, b, e, f, w in cand if math.hypot(e - a, f - b) > 0.03]
    if cc and all(clear_seg(a, b, e, f, w, up["net"], 0.14) for a, b, e, f, w in cc):
        for a, b, e, f, w in cc:
            segs.append((a, b, e, f, w, up["code"]))
        routed.append((up["net"], up["num"], near["ref"], near["num"], [s[4] for s in cc]))

if segs:
    slines = [
        f"  (segment (start {a:.4f} {b:.4f}) (end {e:.4f} {f:.4f}) (width {w:.2f}) (layer \"F.Cu\") (net {nc}))"
        for a, b, e, f, w, nc in segs
    ]
    lp = txt.rfind(")")
    txt = txt[:lp] + "\n" + "\n".join(slines) + "\n)\n"
    pcb.write_text(txt, encoding="utf-8")

print("Routed nets:", len(routed), "Total segments:", len(segs))
for r in routed[:25]:
    print(" ", r)
