import math, re, shutil
from datetime import datetime
from pathlib import Path

PCB = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BDIR = Path("backups")
WIDTH = 0.15
BEYOND_CRT = 1.0
BRANCH_LEN = 1.0
MIN_CLR = 0.25


def backup():
    BDIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BDIR / f"{PCB.stem}_before_nrst_1mm_{ts}.kicad_pcb"
    shutil.copy2(PCB, out)
    print(f"[BACKUP] {out}")


def parse_fps(txt):
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
        cx, cy = [], []
        for ln in blk.splitlines():
            if '(layer "F.Courtyard")' in ln:
                for qm in re.finditer(r'\((?:xy|start|end)\s+([-\d.]+)\s+([-\d.]+)\)', ln):
                    lx, ly = float(qm.group(1)), float(qm.group(2))
                    cx.append(fx + lx * math.cos(rot) - ly * math.sin(rot))
                    cy.append(fy + lx * math.sin(rot) + ly * math.cos(rot))
        if cx and cy:
            box = (min(cx), min(cy), max(cx), max(cy))
        elif xs and ys:
            box = (min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5)
        else:
            box = (fx - 1.0, fy - 1.0, fx + 1.0, fy + 1.0)
        fps.append({"ref": ref, "pads": pads, "box": box})
    return fps


def seg_box_dist(a, b, box):
    x1, y1, x2, y2 = box
    best = 1e9
    for k in range(51):
        t = k / 50.0
        px = a[0] + t * (b[0] - a[0])
        py = a[1] + t * (b[1] - a[1])
        d = math.hypot(max(x1 - px, 0.0, px - x2), max(y1 - py, 0.0, py - y2))
        if d < best:
            best = d
    return best


def strip_old_nrst(txt):
    out, idx, n = [], 0, len(txt)
    while True:
        s = txt.find("(segment", idx)
        if s == -1:
            out.append(txt[idx:])
            break
        out.append(txt[idx:s])
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
        if '(net "/NRST")' not in blk and '(net "NRST")' not in blk:
            out.append(blk)
        idx = e + 1
    return "".join(out)


def main():
    txt = PCB.read_text(encoding="utf-8")
    if txt.count("(") != txt.count(")"):
        raise SystemExit("RULE_ERROR: Unbalanced parentheses in PCB.")

    fps = parse_fps(txt)
    u2 = next((f for f in fps if f["ref"] == "U2"), None)
    if not u2 or "7" not in u2["pads"]:
        raise SystemExit("RULE_ERROR: U2 pin 7 not found.")

    px, py = u2["pads"]["7"]
    min_x, _, max_x, _ = u2["box"]
    if abs(px - min_x) <= abs(max_x - px):
        hub_x, dir_x = min_x - BEYOND_CRT, -1.0
    else:
        hub_x, dir_x = max_x + BEYOND_CRT, 1.0
    hub = (hub_x, py)
    base_ang = 0.0 if dir_x > 0 else math.pi

    up = (hub[0] + BRANCH_LEN * math.cos(base_ang - math.pi / 4), hub[1] + BRANCH_LEN * math.sin(base_ang - math.pi / 4))
    dn = (hub[0] + BRANCH_LEN * math.cos(base_ang + math.pi / 4), hub[1] + BRANCH_LEN * math.sin(base_ang + math.pi / 4))

    segs = [
        ("exit_stub", (px, py), hub),
        ("branch_up_1mm", hub, up),
        ("branch_down_1mm", hub, dn),
    ]

    errors = []
    req_clr = MIN_CLR + WIDTH * 0.5
    for name, a, b in segs:
        for f in fps:
            if f["ref"] == "U2":
                continue
            d = seg_box_dist(a, b, f["box"])
            if d < req_clr:
                errors.append(f"{name} ({a[0]:.3f},{a[1]:.3f})->({b[0]:.3f},{b[1]:.3f}) violates clearance to {f['ref']} (dist={d:.3f} mm < {req_clr:.3f} mm)")

    if errors:
        print("RULE_GATE_FAILED: Aborting without modifying PCB.")
        for e in errors:
            print("  [ERROR]", e)
        raise SystemExit(1)

    backup()
    cleaned = strip_old_nrst(txt)
    lines = "\n".join(
        f'  (segment (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) (width {WIDTH:.2f}) (layer "F.Cu") (net "/NRST"))'
        for _, a, b in segs
    )
    pos = cleaned.rfind(")")
    out = cleaned[:pos] + "\n" + lines + "\n" + cleaned[pos:]
    if out.count("(") != out.count(")"):
        raise SystemExit("RULE_ERROR: Unbalanced parentheses after update.")
    PCB.write_text(out, encoding="utf-8")
    print("OK: Wrote exit stub + 2 branches (1.0 mm each, 0.15 mm width) safely.")
    for name, a, b in segs:
        print(f"  {name}: ({a[0]:.4f}, {a[1]:.4f}) -> ({b[0]:.4f}, {b[1]:.4f}) | len={math.hypot(b[0]-a[0], b[1]-a[1]):.4f} mm")


if __name__ == "__main__":
    main()
