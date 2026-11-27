import math
import re
import shutil
from datetime import datetime
from pathlib import Path

PCB_PATH = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BACKUP_DIR = Path("backups")
NET_NAME = "/NRST"

W_030 = 0.30
MIN_CLEARANCE = 0.25
COURTYARD_PAD = 0.45


def make_backup() -> Path:
    BACKUP_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BACKUP_DIR / f"{PCB_PATH.stem}_before_nrst_030_net_{ts}.kicad_pcb"
    shutil.copy2(PCB_PATH, out)
    if not out.exists() or out.stat().st_size == 0:
        raise RuntimeError(f"Backup failed: {out}")
    print(f"[BACKUP] {out}")
    return out


def paren_balance(txt: str) -> int:
    return txt.count("(") - txt.count(")")


def parse_footprints(txt: str):
    fps = []
    idx, n = 0, len(txt)
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
        blk = txt[s : e + 1]
        idx = e + 1

        rm = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', blk)
        ref = rm.group(1) if rm else "?"

        am = re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)', blk)
        fx = float(am.group(1)) if am else 0.0
        fy = float(am.group(2)) if am else 0.0
        rot = math.radians(float(am.group(3))) if (am and am.group(3)) else 0.0

        pads = {}
        pad_nets = {}
        xs, ys = [], []
        for pm in re.finditer(
            r'\(pad\s+"([^"]+)"\s+\S+\s+\S+\s+\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+[-\d.]+)?\)(.*?)\n\s*\)',
            blk,
            re.DOTALL,
        ):
            pin = pm.group(1)
            lx, ly = float(pm.group(2)), float(pm.group(3))
            pbody = pm.group(4)
            gx = fx + lx * math.cos(rot) - ly * math.sin(rot)
            gy = fy + lx * math.sin(rot) + ly * math.cos(rot)
            pads[pin] = (gx, gy)
            xs.append(gx)
            ys.append(gy)
            nm = re.search(r'\(net\s+\d+\s+"([^"]+)"\)', pbody)
            if nm:
                pad_nets[pin] = nm.group(1)

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
            box = (min(xs) - COURTYARD_PAD, min(ys) - COURTYARD_PAD, max(xs) + COURTYARD_PAD, max(ys) + COURTYARD_PAD)
        else:
            box = (fx - 1.0, fy - 1.0, fx + 1.0, fy + 1.0)

        fps.append({"ref": ref, "pads": pads, "pad_nets": pad_nets, "box": box})
    return fps


def parse_existing_nrst(txt: str):
    segs = []
    for m in re.finditer(
        r'\(segment\s+\(start\s+([-\d.]+)\s+([-\d.]+)\)\s+\(end\s+([-\d.]+)\s+([-\d.]+)\)\s+\(width\s+([-\d.]+)\)\s+\(layer\s+"([^"]+)"\)\s+\(net\s+"(?:/NRST|NRST)"\)',
        txt,
    ):
        segs.append((
            (float(m.group(1)), float(m.group(2))),
            (float(m.group(3)), float(m.group(4))),
            float(m.group(5)),
            m.group(6),
        ))
    return segs


def find_branch_tips(segs, pad7):
    # Identify upper and lower branch tips from existing 0.15 mm segments
    tips = []
    for p1, p2, w, lyr in segs:
        if abs(w - 0.15) > 1e-4:
            continue
        if math.hypot(p1[0] - pad7[0], p1[1] - pad7[1]) < 1e-3:
            continue
        if abs(p2[1] - p1[1]) > 0.05:
            tips.append((p2, lyr))
    if len(tips) < 2:
        raise RuntimeError("Could not find both upper and lower 1.0 mm branch tips on /NRST.")
    tips.sort(key=lambda item: item[0][1])  # smallest Y = upper in KiCad, largest Y = lower
    return tips[0], tips[-1]


def find_next_switch_pin2(fps, sw1_pos):
    # Prefer SW2 pin 2; otherwise nearest non-SW1 pin "2" or NRST pad on another switch
    sw2 = next((f for f in fps if f["ref"] == "SW2" and "2" in f["pads"]), None)
    if sw2:
        return "SW2", "2", sw2["pads"]["2"]

    candidates = []
    for f in fps:
        if f["ref"] == "SW1":
            continue
        if f["ref"].startswith("SW") and "2" in f["pads"]:
            pt = f["pads"]["2"]
            candidates.append((math.hypot(pt[0] - sw1_pos[0], pt[1] - sw1_pos[1]), f["ref"], "2", pt))
        else:
            for pin, pnet in f["pad_nets"].items():
                if pnet in ("/NRST", "NRST") and pin in f["pads"]:
                    pt = f["pads"][pin]
                    candidates.append((math.hypot(pt[0] - sw1_pos[0], pt[1] - sw1_pos[1]), f["ref"], pin, pt))

    if not candidates:
        raise RuntimeError("Next switch pin 2 not found.")
    candidates.sort(key=lambda x: x[0])
    _, ref, pin, pt = candidates[0]
    return ref, pin, pt


def seg_box_dist(a, b, box) -> float:
    x1, y1, x2, y2 = box
    best = float("inf")
    for k in range(61):
        t = k / 60.0
        px = a[0] + t * (b[0] - a[0])
        py = a[1] + t * (b[1] - a[1])
        d = math.hypot(max(x1 - px, 0.0, px - x2), max(y1 - py, 0.0, py - y2))
        if d < best:
            best = d
    return best


def seg_seg_dist(a1, a2, b1, b2) -> float:
    best = float("inf")
    bx, by = b2[0] - b1[0], b2[1] - b1[1]
    b_len2 = bx * bx + by * by
    for k in range(61):
        t = k / 60.0
        px = a1[0] + t * (a2[0] - a1[0])
        py = a1[1] + t * (a2[1] - a1[1])
        if b_len2 < 1e-12:
            d = math.hypot(px - b1[0], py - b1[1])
        else:
            u = max(0.0, min(1.0, ((px - b1[0]) * bx + (py - b1[1]) * by) / b_len2))
            d = math.hypot(px - (b1[0] + u * bx), py - (b1[1] + u * by))
        if d < best:
            best = d
    return best


def plan_orthogonal_030(start_pt, end_pt, fps, allowed_refs, corridor_y):
    sx, sy = start_pt
    ex, ey = end_pt
    for _ in range(40):
        pts = [(sx, sy)]
        if abs(corridor_y - sy) > 1e-3:
            pts.append((sx, corridor_y))
        if abs(ex - sx) > 1e-3:
            pts.append((ex, corridor_y))
        if abs(ey - corridor_y) > 1e-3:
            pts.append((ex, ey))
        if len(pts) == 1:
            pts.append((ex, ey))

        clean_pts = [pts[0]]
        for pt in pts[1:]:
            if math.hypot(pt[0] - clean_pts[-1][0], pt[1] - clean_pts[-1][1]) > 1e-4:
                clean_pts.append(pt)

        blocker = None
        for i in range(len(clean_pts) - 1):
            for fp in fps:
                if fp["ref"] in allowed_refs:
                    continue
                d = seg_box_dist(clean_pts[i], clean_pts[i + 1], fp["box"])
                if d < MIN_CLEARANCE + W_030 * 0.5:
                    blocker = fp["box"]
                    break
            if blocker:
                break

        if not blocker:
            return [
                (clean_pts[i], clean_pts[i + 1], W_030)
                for i in range(len(clean_pts) - 1)
            ]

        _, by1, _, by2 = blocker
        up_y = by1 - (MIN_CLEARANCE + W_030 * 0.5 + 0.15)
        dn_y = by2 + (MIN_CLEARANCE + W_030 * 0.5 + 0.15)
        corridor_y = up_y if abs(corridor_y - up_y) < abs(corridor_y - dn_y) else dn_y

    raise RuntimeError(f"Could not find collision-free 0.30 mm path from {start_pt} to {end_pt}.")


def run_rule_gate(new_segs, fps, all_allowed_refs):
    violations = []

    # 1) Width check
    for idx, (p1, p2, w, lyr, lbl) in enumerate(new_segs, 1):
        if abs(w - W_030) > 1e-6:
            violations.append(f"Seg #{idx} [{lbl}] width {w:.2f} != {W_030:.2f} mm")

    # 2) Clearance against all non-endpoint footprint courtyards & pads
    req_box_clr = MIN_CLEARANCE + W_030 * 0.5
    for idx, (p1, p2, w, lyr, lbl) in enumerate(new_segs, 1):
        for fp in fps:
            if fp["ref"] in all_allowed_refs:
                continue
            d = seg_box_dist(p1, p2, fp["box"])
            if d < req_box_clr:
                violations.append(
                    f"Seg #{idx} [{lbl}] violates clearance to {fp['ref']} courtyard: {d:.3f} mm < {req_box_clr:.3f} mm"
                )
            for pin, pad_xy in fp["pads"].items():
                dp = seg_seg_dist(p1, p2, pad_xy, pad_xy)
                if dp < req_box_clr:
                    violations.append(
                        f"Seg #{idx} [{lbl}] too close to {fp['ref']} pin {pin}: {dp:.3f} mm < {req_box_clr:.3f} mm"
                    )

    # 3) Clearance between non-adjacent new segments (avoid accidental shorts/crowding)
    req_seg_clr = MIN_CLEARANCE + W_030
    for i in range(len(new_segs)):
        for j in range(i + 1, len(new_segs)):
            a1, a2, _, _, _ = new_segs[i]
            b1, b2, _, _, _ = new_segs[j]
            touches_endpoint = (
                math.hypot(a2[0] - b1[0], a2[1] - b1[1]) < 1e-3
                or math.hypot(a1[0] - b2[0], a1[1] - b2[1]) < 1e-3
                or math.hypot(a1[0] - b1[0], a1[1] - b1[1]) < 1e-3
                or math.hypot(a2[0] - b2[0], a2[1] - b2[1]) < 1e-3
            )
            if touches_endpoint:
                continue
            d = seg_seg_dist(a1, a2, b1, b2)
            if d < req_seg_clr:
                violations.append(f"New segments #{i+1} and #{j+1} too close: {d:.3f} mm < {req_seg_clr:.3f} mm")

    return violations


def main() -> None:
    if not PCB_PATH.exists():
        raise FileNotFoundError(f"Missing PCB file: {PCB_PATH}")

    make_backup()
    txt = PCB_PATH.read_text(encoding="utf-8")
    if paren_balance(txt) != 0:
        raise RuntimeError("Unbalanced parentheses before editing.")

    fps = parse_footprints(txt)
    fp_map = {f["ref"]: f for f in fps}

    for req_ref, req_pin in [("U2", "7"), ("C17", "2"), ("SW1", "2"), ("J2", "5")]:
        if req_ref not in fp_map or req_pin not in fp_map[req_ref]["pads"]:
            raise RuntimeError(f"Required pad {req_ref}.{req_pin} not found in PCB.")

    pad_u2_7 = fp_map["U2"]["pads"]["7"]
    pad_c17_2 = fp_map["C17"]["pads"]["2"]
    pad_sw1_2 = fp_map["SW1"]["pads"]["2"]
    pad_j2_5 = fp_map["J2"]["pads"]["5"]
    next_sw_ref, next_sw_pin, pad_next_sw = find_next_switch_pin2(fps, pad_sw1_2)

    existing_nrst = parse_existing_nrst(txt)
    (up_tip, up_layer), (dn_tip, dn_layer) = find_branch_tips(existing_nrst, pad_u2_7)

    # Lower branch: branch_down_tip -> C17.2 -> SW1.2 -> next_sw.pin2 (all 0.30 mm)
    low_1 = plan_orthogonal_030(dn_tip, pad_c17_2, fps, {"U2", "C17"}, dn_tip[1] + 0.6)
    low_2 = plan_orthogonal_030(pad_c17_2, pad_sw1_2, fps, {"C17", "SW1"}, pad_c17_2[1] + 0.6)
    low_3 = plan_orthogonal_030(pad_sw1_2, pad_next_sw, fps, {"SW1", next_sw_ref}, pad_sw1_2[1] + 0.6)

    # Upper branch: branch_up_tip -> J2.5 (0.30 mm, clear of all components)
    up_1 = plan_orthogonal_030(up_tip, pad_j2_5, fps, {"U2", "J2"}, up_tip[1] - 0.8)

    new_segs = []
    for p1, p2, w in low_1:
        new_segs.append((p1, p2, w, dn_layer, "lower->C17.2"))
    for p1, p2, w in low_2:
        new_segs.append((p1, p2, w, dn_layer, "C17.2->SW1.2"))
    for p1, p2, w in low_3:
        new_segs.append((p1, p2, w, dn_layer, f"SW1.2->{next_sw_ref}.{next_sw_pin}"))
    for p1, p2, w in up_1:
        new_segs.append((p1, p2, w, up_layer, "upper->J2.5"))

    allowed_refs = {"U2", "C17", "SW1", next_sw_ref, "J2"}
    violations = run_rule_gate(new_segs, fps, allowed_refs)
    if violations:
        print("RULE_GATE_FAILED: No tracks written to PCB.")
        for v in violations:
            print("  [RULE ERROR]", v)
        raise SystemExit(1)

    print("RULE_GATE_PASSED: All clearance, no-collision, width (0.30 mm), and spacing rules passed.")

    add_lines = "\n".join(
        f'  (segment (start {p1[0]:.4f} {p1[1]:.4f}) (end {p2[0]:.4f} {p2[1]:.4f}) '
        f'(width {w:.2f}) (layer "{lyr}") (net "{NET_NAME}"))'
        for p1, p2, w, lyr, _ in new_segs
    )

    pos = txt.rfind(")")
    if pos == -1:
        raise RuntimeError("Invalid .kicad_pcb file format.")

    updated = txt[:pos] + "\n" + add_lines + "\n" + txt[pos:]
    if paren_balance(updated) != 0:
        raise RuntimeError("Unbalanced parentheses after update.")

    PCB_PATH.write_text(updated, encoding="utf-8")
    print(f"Added {len(new_segs)} segments on net {NET_NAME} | Parenthesis balance: {paren_balance(updated)}")
    for idx, (p1, p2, w, lyr, lbl) in enumerate(new_segs, 1):
        print(f"  #{idx} [{lbl}] ({lyr}): ({p1[0]:.4f}, {p1[1]:.4f}) -> ({p2[0]:.4f}, {p2[1]:.4f}) | w={w:.2f} mm")


if __name__ == "__main__":
    main()
