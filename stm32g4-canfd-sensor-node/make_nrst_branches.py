import math, re, shutil
from datetime import datetime
from pathlib import Path

PCB = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BDIR = Path("backups")
WIDTH = 0.15
LEN_MM = 1.0
MIN_CLR = 0.25

def backup():
    BDIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BDIR / f"{PCB.stem}_safe_2br_{ts}.kicad_pcb"
    shutil.copy2(PCB, out)
    print(f"[BACKUP] {out}")

def get_fps(txt):
    fps, idx, n = [], 0, len(txt)
    while True:
        s = txt.find("(footprint ", idx)
        if s == -1:
            break
        d, e = 0, s
        while e < n:
            if txt[e] == "(":
                d += 1
            elif txt[e] == ")":
                d -= 1
                if d == 0:
                    break
            e += 1
        blk = txt[s:e + 1]
        idx = e + 1
        rm = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', blk)
        ref = rm.group(1) if rm else "?"
        am = re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)', blk)
        fx, fy = float(am.group(1)), float(am.group(2))
        rot = math.radians(float(am.group(3))) if (am and am.group(3)) else 0.0
        pads, xs, ys = {}, [], []
        for pm in re.finditer(r'\(pad\s+"([^"]+)"\s+\S+\s+\S+\s+\(at\s+([-\d.]+)\s+([-\d.]+)', blk):
            lx, ly = float(pm.group(2)), float(pm.group(3))
            gx = fx + lx * math.cos(rot) - ly * math.sin(rot)
            gy = fy + lx * math.sin(rot) + ly * math.cos(rot)
            pads[pm.group(1)] = (gx, gy)
            xs.append(gx)
            ys.append(gy)
        box = (min(xs) - 0.4, min(ys) - 0.4, max(xs) + 0.4, max(ys) + 0.4) if xs else (fx - 1.0, fy - 1.0, fx + 1.0, fy + 1.0)
        fps.append({"ref": ref, "pads": pads, "box": box})
    return fps

def seg_box_dist(a, b, box):
    x1, y1, x2, y2 = box
    best = 1e9
    for k in range(41):
        t = k / 40.0
        px = a[0] + t * (b[0] - a[0])
        py = a[1] + t * (b[1] - a[1])
        d = math.hypot(max(x1 - px, 0.0, px - x2), max(y1 - py, 0.0, py - y2))
        if d < best:
            best = d
    return best

def main():
    txt = PCB.read_text(encoding="utf-8")
    if txt.count("(") != txt.count(")"):
        raise SystemExit("RULE_ERROR: Unbalanced parentheses in PCB.")

    fps = get_fps(txt)
    u2 = next((f for f in fps if f["ref"] == "U2"), None)
    if not u2 or "7" not in u2["pads"]:
        raise SystemExit("RULE_ERROR: U2 pin 7 not found.")
    pad7 = u2["pads"]["7"]

    segs = [
        (float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), m.group(6))
        for m in re.finditer(
            r'\(segment\s+\(start\s+([-\d.]+)\s+([-\d.]+)\)\s+\(end\s+([-\d.]+)\s+([-\d.]+)\)\s+\(width\s+([-\d.]+)\)\s+\(layer\s+"([^"]+)"\)\s+\(net\s+"(?:/NRST|NRST)"\)\)',
            txt,
        )
    ]
    if not segs:
        raise SystemExit("RULE_ERROR: No /NRST stub found on U2 pin 7. Aborting without drawing.")

    p1, p2, w, lyr = min(
        segs,
        key=lambda s: min(math.hypot(s[0] - pad7[0], s[1] - pad7[1]), math.hypot(s[2] - pad7[0], s[3] - pad7[1])),
    )
    hub = p2 if math.hypot(p1[0] - pad7[0], p1[1] - pad7[1]) <= math.hypot(p2[0] - pad7[0], p2[1] - pad7[1]) else p1
    base = math.atan2(hub[1] - pad7[1], hub[0] - pad7[0]) if math.hypot(hub[0] - pad7[0], hub[1] - pad7[1]) > 1e-6 else math.pi

    up = (hub[0] + LEN_MM * math.cos(base - math.pi / 4), hub[1] + LEN_MM * math.sin(base - math.pi / 4))
    dn = (hub[0] + LEN_MM * math.cos(base + math.pi / 4), hub[1] + LEN_MM * math.sin(base + math.pi / 4))
    branches = [("up_1mm", hub, up), ("dn_1mm", hub, dn)]

    errors = []
    req_clr = MIN_CLR + WIDTH * 0.5
    for name, a, b in branches:
        if abs(math.hypot(b[0] - a[0], b[1] - a[1]) - LEN_MM) > 1e-3:
            errors.append(f"{name} length != {LEN_MM} mm")
        for f in fps:
            if f["ref"] == "U2":
                continue
            ref_name = f["ref"]
            d = seg_box_dist(a, b, f["box"])
            if d < req_clr:
                errors.append(f"{name} violates clearance to {ref_name} (dist={d:.3f} mm < {req_clr:.3f} mm)")

    if errors:
        print("RULE_GATE_FAILED: No tracks drawn.")
        for e in errors:
            print("  [ERROR]", e)
        raise SystemExit(1)

    backup()
    add = "\n".join(
        f'  (segment (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) (width {WIDTH:.2f}) (layer "{lyr}") (net "/NRST"))'
        for _, a, b in branches
    )
    pos = txt.rfind(")")
    out = txt[:pos] + "\n" + add + "\n" + txt[pos:]
    if out.count("(") != out.count(")"):
        raise SystemExit("RULE_ERROR: Unbalanced parentheses after edit.")
    PCB.write_text(out, encoding="utf-8")
    print("OK: Added 2 branches (1.0 mm up & 1.0 mm down, 0.15 mm width) safely.")
    for name, a, b in branches:
        print(f"  {name}: ({a[0]:.4f}, {a[1]:.4f}) -> ({b[0]:.4f}, {b[1]:.4f})")

if __name__ == "__main__":
    main()
