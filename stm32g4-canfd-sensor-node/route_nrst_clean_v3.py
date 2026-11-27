import math
import re
import shutil
from datetime import datetime
from pathlib import Path

PCB_PATH = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BACKUP_DIR = Path("backups")
NET_NAME = "/NRST"

SRC_REF = "U2"
SRC_PIN = "7"

CONN_REF = "J2"
CONN_PIN = "5"

# Primary target capacitor (C1 pin 2; fallback to C17 pin 2 if C1 is absent)
CAP_TARGETS = [("C1", "2"), ("C17", "2")]

W_015 = 0.15
W_030 = 0.30
CLEARANCE_PAD = 0.65
EXIT_STUB = 0.60


def backup_stage(stage_label: str) -> Path:
    BACKUP_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BACKUP_DIR / f"{PCB_PATH.stem}_{stage_label}_{ts}.kicad_pcb"
    shutil.copy2(PCB_PATH, out)
    if not out.exists() or out.stat().st_size == 0:
        raise RuntimeError(f"Failed to create backup: {out}")
    print(f"[BACKUP] {out}")
    return out


def paren_balance(text: str) -> int:
    return text.count("(") - text.count(")")


def parse_footprints(text: str):
    fps = []
    idx = 0
    n = len(text)
    while True:
        start = text.find("(footprint ", idx)
        if start == -1:
            break
        depth = 0
        end = start
        while end < n:
            if text[end] == "(":
                depth += 1
            elif text[end] == ")":
                depth -= 1
                if depth == 0:
                    break
            end += 1
        block = text[start : end + 1]
        idx = end + 1

        ref_m = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', block)
        ref = ref_m.group(1) if ref_m else "UNKNOWN"

        at_m = re.search(r"\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)", block)
        fx = float(at_m.group(1)) if at_m else 0.0
        fy = float(at_m.group(2)) if at_m else 0.0
        rot = math.radians(float(at_m.group(3))) if (at_m and at_m.group(3)) else 0.0

        pads = {}
        pad_nets = {}
        xs, ys = [], []
        for pm in re.finditer(
            r'\(pad\s+"([^"]+)"\s+\S+\s+\S+\s+\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+[-\d.]+)?\)(.*?)\n\s*\)',
            block,
            re.DOTALL,
        ):
            pin = pm.group(1)
            lx = float(pm.group(2))
            ly = float(pm.group(3))
            pbody = pm.group(4)
            gx = fx + lx * math.cos(rot) - ly * math.sin(rot)
            gy = fy + lx * math.sin(rot) + ly * math.cos(rot)
            pads[pin] = (gx, gy)
            xs.append(gx)
            ys.append(gy)

            nm = re.search(r'\(net\s+[^)]+\)', pbody)
            if nm:
                pad_nets[pin] = nm.group(0)

        if xs and ys:
            box = (
                min(xs) - CLEARANCE_PAD,
                min(ys) - CLEARANCE_PAD,
                max(xs) + CLEARANCE_PAD,
                max(ys) + CLEARANCE_PAD,
            )
        else:
            box = (fx - 1.5, fy - 1.5, fx + 1.5, fy + 1.5)

        fps.append({"ref": ref, "pads": pads, "pad_nets": pad_nets, "box": box})
    return fps


def seg_intersects_box(p1, p2, box) -> bool:
    min_x, min_y, max_x, max_y = box
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    t0, t1 = 0.0, 1.0
    for p, q in [(-dx, x1 - min_x), (dx, max_x - x1), (-dy, y1 - min_y), (dy, max_y - y1)]:
        if abs(p) < 1e-12:
            if q < 0:
                return False
        else:
            t = q / p
            if p < 0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t
    return t0 <= t1


def interior_angle_deg(a, b, c) -> float:
    # Angle between vector b->a and vector b->c
    v1x, v1y = a[0] - b[0], a[1] - b[1]
    v2x, v2y = c[0] - b[0], c[1] - b[1]
    n1 = math.hypot(v1x, v1y)
    n2 = math.hypot(v2x, v2y)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    val = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (n1 * n2)))
    return math.degrees(math.acos(val))


def enforce_non_acute_polyline(pts):
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        p_prev = out[-1]
        p_cur = pts[i]
        p_nxt = pts[i + 1]
        if interior_angle_deg(p_prev, p_cur, p_nxt) < 90.0:
            # Smooth acute bend into a gentle chamfer/split point
            mx = (p_prev[0] + p_cur[0] + p_nxt[0]) / 3.0
            my = (p_prev[1] + p_cur[1] + p_nxt[1]) / 3.0
            out.append((mx, my))
        else:
            out.append(p_cur)
    out.append(pts[-1])
    return out


def route_branch_avoiding_boxes(start_xy, end_xy, init_corridor_y, fps, ignore_refs, force_015_only: bool, allow_030_mid: bool):
    sx, sy = start_xy
    ex, ey = end_xy
    dx_sign = -1.0 if ex < sx else 1.0
    corridor_y = init_corridor_y

    pts = [start_xy, end_xy]
    for _ in range(30):
        p0 = (sx, sy)
        p1 = (sx + dx_sign * EXIT_STUB, sy)
        p2 = (sx + dx_sign * (EXIT_STUB + abs(corridor_y - sy)), corridor_y)
        p3 = (ex - dx_sign * (EXIT_STUB + abs(ey - corridor_y)), corridor_y)
        p4 = (ex - dx_sign * EXIT_STUB, ey)
        p5 = (ex, ey)

        raw = []
        for pt in [p0, p1, p2, p3, p4, p5]:
            if not raw or math.hypot(pt[0] - raw[-1][0], pt[1] - raw[-1][1]) > 1e-4:
                raw.append(pt)
        pts = enforce_non_acute_polyline(raw)

        blocker = None
        for i in range(len(pts) - 1):
            for fp in fps:
                if fp["ref"] in ignore_refs:
                    continue
                if seg_intersects_box(pts[i], pts[i + 1], fp["box"]):
                    blocker = fp["box"]
                    break
            if blocker:
                break

        if not blocker:
            break

        _, by1, _, by2 = blocker
        up_y = by1 - 0.35
        dn_y = by2 + 0.35
        corridor_y = up_y if abs(corridor_y - up_y) < abs(corridor_y - dn_y) else dn_y

    segs = []
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 1e-4:
            continue

        if force_015_only or i == 0:
            width = W_015
        elif allow_030_mid and i not in (0, len(pts) - 2) and seg_len >= 1.5:
            width = W_030
        else:
            width = W_015

        segs.append(((x1, y1), (x2, y2), width))

    # Verify no segment has an acute angle (< 90 deg)
    for i in range(len(pts) - 2):
        ang = interior_angle_deg(pts[i], pts[i + 1], pts[i + 2])
        if ang < 90.0:
            raise RuntimeError(f"Acute angle detected ({ang:.1f} deg) in branch.")

    return segs


def remove_old_nrst_segments(text: str) -> tuple[str, int]:
    out = []
    idx = 0
    n = len(text)
    removed = 0
    while True:
        start = text.find("(segment", idx)
        if start == -1:
            out.append(text[idx:])
            break
        out.append(text[idx:start])
        depth = 0
        end = start
        while end < n:
            if text[end] == "(":
                depth += 1
            elif text[end] == ")":
                depth -= 1
                if depth == 0:
                    break
            end += 1
        block = text[start : end + 1]
        if '(net "/NRST")' in block or '(net "NRST")' in block:
            removed += 1
        else:
            out.append(block)
        idx = end + 1
    return "".join(out), removed


def format_segments(seg_tuples, net_name: str, layer: str = "F.Cu") -> list[str]:
    return [
        f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) '
        f'(width {w:.2f}) (layer "{layer}") (net "{net_name}"))'
        for (x1, y1), (x2, y2), w in seg_tuples
    ]


def main() -> None:
    if not PCB_PATH.exists():
        raise FileNotFoundError(f"Missing PCB file: {PCB_PATH}")

    # Step 0: Dedicated backup before modifying NRST
    backup_stage("before_step1_nrst_clean")

    text = PCB_PATH.read_text(encoding="utf-8")
    if paren_balance(text) != 0:
        raise RuntimeError("Unbalanced parentheses before editing.")

    fps = parse_footprints(text)
    fp_map = {fp["ref"]: fp for fp in fps}

    if SRC_REF not in fp_map or SRC_PIN not in fp_map[SRC_REF]["pads"]:
        raise RuntimeError(f"Missing {SRC_REF} pin {SRC_PIN}")
    if CONN_REF not in fp_map or CONN_PIN not in fp_map[CONN_REF]["pads"]:
        raise RuntimeError(f"Missing {CONN_REF} pin {CONN_PIN}")

    cap_ref, cap_pin = None, None
    for cref, cpin in CAP_TARGETS:
        if cref in fp_map and cpin in fp_map[cref]["pads"]:
            net_info = fp_map[cref]["pad_nets"].get(cpin, "")
            if "GND" in net_info.upper():
                raise RuntimeError(f"Safety halt: {cref} pin {cpin} is connected to GND ({net_info}).")
            cap_ref, cap_pin = cref, cpin
            break

    if not cap_ref:
        raise RuntimeError("No valid non-GND capacitor target pin found.")

    p_u2_7 = fp_map[SRC_REF]["pads"][SRC_PIN]
    p_conn_5 = fp_map[CONN_REF]["pads"][CONN_PIN]
    p_cap = fp_map[cap_ref]["pads"][cap_pin]

    print(f"U2.7={p_u2_7} | {CONN_REF}.{CONN_PIN}={p_conn_5} | {cap_ref}.{cap_pin}={p_cap}")

    # Branch A: U2.7 -> J2.5 (0.15 mm exit, non-acute split, 0.30 mm in open corridor, clear of other branch)
    segs_j2 = route_branch_avoiding_boxes(
        p_u2_7,
        p_conn_5,
        init_corridor_y=(p_u2_7[1] + p_conn_5[1]) * 0.5 - 1.5,
        fps=fps,
        ignore_refs={SRC_REF, CONN_REF},
        force_015_only=False,
        allow_030_mid=True,
    )

    # Branch B: U2.7 -> Cap pin (0.15 mm throughout, non-acute split, strictly avoiding GND/C15/other parts)
    segs_cap = route_branch_avoiding_boxes(
        p_u2_7,
        p_cap,
        init_corridor_y=(p_u2_7[1] + p_cap[1]) * 0.5 + 1.5,
        fps=fps,
        ignore_refs={SRC_REF, cap_ref},
        force_015_only=True,
        allow_030_mid=False,
    )

    all_segs = segs_j2 + segs_cap
    cleaned, removed_cnt = remove_old_nrst_segments(text)
    block = "\n" + "\n".join(format_segments(all_segs, net_name=NET_NAME)) + "\n"

    end_pos = cleaned.rfind(")")
    if end_pos == -1:
        raise RuntimeError("Invalid .kicad_pcb file structure.")

    updated = cleaned[:end_pos] + block + cleaned[end_pos:]
    if paren_balance(updated) != 0:
        raise RuntimeError("Unbalanced parentheses after editing.")

    PCB_PATH.write_text(updated, encoding="utf-8")
    print(f"Removed old /NRST segments: {removed_cnt} | Added clean segments: {len(all_segs)}")
    print(f"Parenthesis balance: {paren_balance(updated)}")


if __name__ == "__main__":
    main()
