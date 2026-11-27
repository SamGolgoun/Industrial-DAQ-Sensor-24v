import math
import pathlib
import re

pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = pcb.read_text(encoding="utf-8")


def find_end(s: str, idx: int) -> int:
d = 0
q = False
esc = False
for i in range(idx, len(s)):
    c = s[i]
    if esc:
        esc = False
    elif c == "\\":
        esc = True
    elif c == '"':
        q = not q
    elif not q:
        if c == "(":
            d += 1
        elif c == ")":
            d -= 1
            if d == 0:
                return i
return -1


def iter_fps(s: str):
er = find_end(s, 0)
i = 1
while i < er:
    if s[i] == "(":
        e = find_end(s, i)
        if e < 0:
            break
        b = s[i : e + 1]
        if b.startswith("(footprint "):
            yield i, e + 1, b
        i = e + 1
    else:
        i += 1


def parse_fp(st: int, en: int, b: str):
rm = re.search(r'\(\s*property\s+"Reference"\s+"([^"]+)"', b)
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
    elif c == '"':
        q = not q
    elif not q:
        if c == "(":
            if d == 1 and re.match(r"^\(\s*at\b", b[i : i + 10]):
                ae = find_end(b, i)
                am = re.match(
                    r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$",
                    b[i : ae + 1],
                )
                if am:
                    fx, fy = float(am.group(1)), float(am.group(2))
                    fdeg = float(am.group(3)) if am.group(3) else 0.0
                    at_sp = (i, ae + 1)
                    break
            d += 1
        elif c == ")":
            d -= 1
    i += 1

pads = []
pi = 1
while pi < len(b):
    if b[pi : pi + 5] == "(pad ":
        pe = find_end(b, pi)
        if pe < 0:
            break
        pb = b[pi : pe + 1]
        pm = re.match(r'^\(\s*pad\s+"([^"]*)"', pb)
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pb)
        psm = re.search(r"\(\s*size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pb)
        pnm = re.search(r'\(\s*net\s+(\d+)\s+"([^"]*)"', pb)
        if pm and pam and psm:
            pads.append(
                {
                    "num": pm.group(1),
                    "lx": float(pam.group(1)),
                    "ly": float(pam.group(2)),
                    "sw": float(psm.group(1)),
                    "sh": float(psm.group(2)),
                    "code": int(pnm.group(1)) if pnm else 0,
                    "net": pnm.group(2) if pnm else "",
                }
            )
        pi = pe + 1
    else:
        pi += 1

return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp, "pads": pads}


def pxy(fp, pad, cx=None, cy=None, cdeg=None):
fx = fp["x"] if cx is None else cx
fy = fp["y"] if cy is None else cy
deg = fp["deg"] if cdeg is None else cdeg
rad = math.radians(-deg)
return fx + pad["lx"] * math.cos(rad) - pad["ly"] * math.sin(rad), fy + pad["lx"] * math.sin(rad) + pad["ly"] * math.cos(rad)


def pt_seg(px, py, x1, y1, x2, y2):
dx, dy = x2 - x1, y2 - y1
l2 = dx * dx + dy * dy
if l2 == 0:
    return math.hypot(px - x1, py - y1)
t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def ccw(p1, p2, p3):
return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])


def cross(a1, a2, b1, b2):
if a1 in (b1, b2) or a2 in (b1, b2):
    return False
return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))


def seg_dist(a1, a2, b1, b2):
if cross(a1, a2, b1, b2):
    return 0.0
return min(
    pt_seg(a1[0], a1[1], b1[0], b1[1], b2[0], b2[1]),
    pt_seg(a2[0], a2[1], b1[0], b1[1], b2[0], b2[1]),
    pt_seg(b1[0], b1[1], a1[0], a1[1], a2[0], a2[1]),
    pt_seg(b2[0], b2[1], a1[0], a1[1], a2[0], a2[1]),
)


def run():
global txt
fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
by_ref = {f["ref"]: f for f in fps}
u1 = by_ref.get("U1")
if not u1:
    print("U1 not found.")
    return

u1_nets = {p["net"] for p in u1["pads"] if p["net"] and p["net"] != "GND"}
opt_log = []

def score_fp(fp, cx, cy, cdeg):
    tot = 0.0
    for p in fp["pads"]:
        if not p["net"]:
            continue
        wx, wy = pxy(fp, p, cx, cy, cdeg)
        if p["net"] == "GND":
            gs = [pxy(ofp, op) for ofp in fps if ofp["ref"] != fp["ref"] for op in ofp["pads"] if op["net"] == "GND"]
            if gs:
                tot += 0.5 * min(math.hypot(wx - gx, wy - gy) for gx, gy in gs)
        else:
            ms = [pxy(u1, up) for up in u1["pads"] if up["net"] == p["net"]]
            if ms:
                tot += min(math.hypot(wx - ux, wy - uy) for ux, uy in ms)
    return tot

def free_spot(fp, cx, cy, cdeg):
    for p in fp["pads"]:
        wx, wy = pxy(fp, p, cx, cy, cdeg)
        r1 = max(p["sw"], p["sh"]) * 0.5 + 0.35
        for ofp in fps:
            if ofp["ref"] == fp["ref"]:
                continue
            for op in ofp["pads"]:
                ox, oy = pxy(ofp, op)
                if math.hypot(wx - ox, wy - oy) < (r1 + max(op["sw"], op["sh"]) * 0.5 + 0.35):
                    return False
    return True

for fp in fps:
    if not fp["ref"].startswith(("C", "R", "D", "Y", "FB")) or len(fp["pads"]) > 4:
        continue
    if not ({p["net"] for p in fp["pads"] if p["net"]} & u1_nets):
        continue
    base = score_fp(fp, fp["x"], fp["y"], fp["deg"])
    best = (fp["x"], fp["y"], fp["deg"], base)
    for deg in sorted({fp["deg"] % 360.0, 0.0, 90.0, 180.0, 270.0}):
        for dx in (0.0, -0.4, 0.4, -0.8, 0.8):
            for dy in (0.0, -0.4, 0.4, -0.8, 0.8):
                cx, cy = round(fp["x"] + dx, 3), round(fp["y"] + dy, 3)
                if 104.0 <= cx <= 176.0 and 64.0 <= cy <= 116.0 and free_spot(fp, cx, cy, deg):
                    sc = score_fp(fp, cx, cy, deg)
                    if sc + 0.15 < best[3]:
                        best = (cx, cy, deg, sc)
    if (best[0], best[1], best[2]) != (fp["x"], fp["y"], fp["deg"]):
        opt_log.append((fp["ref"], (fp["x"], fp["y"], fp["deg"]), (best[0], best[1], best[2])))
        fp["x"], fp["y"], fp["deg"] = best[0], best[1], best[2]

repls = []
for fp in fps:
    if fp["at_sp"]:
        sa, ea = fp["at_sp"]
        na = f"(at {fp['x']:.3f} {fp['y']:.3f} {fp['deg']:.0f})" if fp["deg"] != 0 else f"(at {fp['x']:.3f} {fp['y']:.3f})"
        repls.append((fp["st"], fp["en"], fp["b"][:sa] + na + fp["b"][ea:]))
for s, e, nb in sorted(repls, key=lambda x: x[0], reverse=True):
    txt = txt[:s] + nb + txt[e:]

all_pads = []
for fp in fps:
    for p in fp["pads"]:
        wx, wy = pxy(fp, p)
        all_pads.append(
            {
                "ref": fp["ref"],
                "num": p["num"],
                "x": wx,
                "y": wy,
                "r": max(p["sw"], p["sh"]) * 0.5,
                "net": p["net"],
                "code": p["code"],
            }
        )

segs = []

def clear_seg(x1, y1, x2, y2, w, net):
    for pd in all_pads:
        if pd["net"] != net and pt_seg(pd["x"], pd["y"], x1, y1, x2, y2) < (pd["r"] + w * 0.5 + 0.16):
            return False
    for sx1, sy1, sx2, sy2, sw, _, _, snet in segs:
        if snet != net and seg_dist((x1, y1), (x2, y2), (sx1, sy1), (sx2, sy2)) < (w * 0.5 + sw * 0.5 + 0.16):
            return False
    return True

def stepped_path(ps, pe):
    x1, y1, x2, y2 = ps["x"], ps["y"], pe["x"], pe["y"]
    if math.hypot(x2 - x1, y2 - y1) < 0.25:
        return []
    vx, vy = x1 - u1["x"], y1 - u1["y"]
    edx, edy = ((1.0 if vx >= 0 else -1.0), 0.0) if abs(vx) >= abs(vy) else (0.0, (1.0 if vy >= 0 else -1.0))
    nlen = min(1.0, max(0.5, math.hypot(x2 - x1, y2 - y1) * 0.2))
    nx1, ny1 = round(x1 + edx * nlen, 4), round(y1 + edy * nlen, 4)
    wide = pe["ref"].startswith(("R", "D", "L", "FB", "F")) or ("3V3" in ps["net"])
    tw = 0.35 if wide else 0.25
    ew = 0.40 if (wide and pe["r"] >= 0.35) else 0.25
    dx, dy = x2 - nx1, y2 - ny1
    diag = min(abs(dx), abs(dy))
    sx, sy = (1.0 if dx >= 0 else -1.0), (1.0 if dy >= 0 else -1.0)
    mx1, my1 = round(nx1 + sx * diag, 4), round(ny1 + sy * diag, 4)
    cands = [
        [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, mx1, my1, tw), (mx1, my1, x2, y2, ew)],
        [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, x2, y2, tw)],
    ]
    for c in cands:
        cc = [(a, b, e, f, w) for a, b, e, f, w in c if math.hypot(e - a, f - b) > 0.03]
        if cc and all(clear_seg(a, b, e, f, w, ps["net"]) for a, b, e, f, w in cc):
            return [(a, b, e, f, w, "F.Cu", ps["code"], ps["net"]) for a, b, e, f, w in cc]
    return []

u1_pins = sorted(
    [p for p in all_pads if p["ref"] == "U1" and p["net"] and p["net"] != "GND"],
    key=lambda x: (0 if "OSC" in x["net"].upper() else 1 if ("ADC" in x["net"].upper() or "ANA" in x["net"].upper()) else 2, x["num"]),
)
log_r = []
for up in u1_pins:
    tgs = [p for p in all_pads if p["net"] == up["net"] and not (p["ref"] == "U1" and p["num"] == up["num"])]
    if not tgs:
        continue
    near = min(tgs, key=lambda tp: math.hypot(tp["x"] - up["x"], tp["y"] - up["y"]))
    p_segs = stepped_path(up, near)
    if p_segs:
        segs.extend(p_segs)
        log_r.append((up["net"], f"U1.{up['num']}->{near['ref']}.{near['num']}", [s[4] for s in p_segs]))

if segs:
    slines = [
        f'  (segment (start {a:.4f} {b:.4f}) (end {e:.4f} {f:.4f}) (width {w:.2f}) (layer "{lay}") (net {nc}))'
        for a, b, e, f, w, lay, nc, _ in segs
    ]
    lp = txt.rfind(")")
    txt = txt[:lp] + "\n" + "\n".join(slines) + "\n)\n"

pcb.write_text(txt, encoding="utf-8")
print("Adjusted footprints:", len(opt_log), opt_log)
print("Routed MCU nets:", len(log_r), "segments:", len(segs))
for row in log_r:
    print(" ", row)


if __name__ == "__main__":
run()
