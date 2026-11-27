import math
import re
from pathlib import Path

PCB_PATH = Path("stm32g4-canfd-sensor-node.kicad_pcb")
NET_NAME = "/NRST"

SRC_REF = "U2"
SRC_PIN = "7"

C15_REF = "C15"
C15_PIN = "2"

C17_REF = "C17"
C17_PIN = "2"

CONN_REF = "J2"
CONN_PIN = "5"

WIDTH_015 = 0.15
WIDTH_022 = 0.22
WIDTH_030 = 0.30
SAFE_MARGIN = 0.55
EXIT_LEN = 0.50


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
        xs, ys = [], []
        for pm in re.finditer(
            r'\(pad\s+"([^"]+)"\s+\S+\s+\S+\s+\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+[-\d.]+)?\)',
            block,
        ):
            pin = pm.group(1)
            lx = float(pm.group(2))
            ly = float(pm.group(3))
            gx = fx + lx * math.cos(rot) - ly * math.sin(rot)
            gy = fy + lx * math.sin(rot) + ly * math.cos(rot)
            pads[pin] = (gx, gy)
            xs.append(gx)
            ys.append(gy)

        if xs and ys:
            box = (min(xs) - SAFE_MARGIN, min(ys) - SAFE_MARGIN, max(xs) + SAFE_MARGIN, max(ys) + SAFE_MARGIN)
        else:
            box = (fx - 1.5, fy - 1.5, fx + 1.5, fy + 1.5)

        fps.append({"ref": ref, "x": fx, "y": fy, "pads": pads, "box": box})
    return fps


def seg_hits_box(p1, p2, box) -> bool:
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


def angle_deg(a, b, c) -> float:
    v1 = (a[0] - b[0], a[1] - b[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    val = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(val))


def sanitize_angles(pts):
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        prev_pt = out[-1]
        cur_pt = pts[i]
        next_pt = pts[i + 1]
        if angle_deg(prev_pt, cur_pt, next_pt) < 90.0:
            mx = (prev_pt[0] + next_pt[0]) * 0.5
            my = (prev_pt[1] + next_pt[1]) * 0.5
            out.append((mx, my))
        else:
            out.append(cur_pt)
    out.append(pts[-1])
    return out


def route_clear_corridor(start_xy, end_xy, initial_y, fps, ignore_refs, allow_030: bool, force_015_end: bool):
    sx, sy = start_xy
    ex, ey = end_xy
    dir_x = 1.0 if ex >= sx else -1.0
    corridor_y = initial_y

    for _ in range(25):
        p0 = (sx, sy)
        p1 = (sx + dir_x * EXIT_LEN, sy)
        p2 = (sx + dir_x * (EXIT_LEN + abs(corridor_y - sy)), corridor_y)
        p3 = (ex - dir_x * (EXIT_LEN + abs(ey - corridor_y)), corridor_y)
        p4 = (ex - dir_x * EXIT_LEN, ey)
        p5 = (ex, ey)

        uniq = []
        for pt in [p0, p1, p2, p3, p4, p5]:
            if not uniq or math.hypot(pt[0] - uniq[-1][0], pt[1] - uniq[-1][1]) > 1e-4:
                uniq.append(pt)
        pts = sanitize_angles(uniq)

        hit_box = None
        for i in range(len(pts) - 1):
            for fp in fps:
                if fp["ref"] in ignore_refs:
                    continue
                if seg_hits_box(pts[i], pts[i + 1], fp["box"]):
                    hit_box = fp["box"]
                    break
            if hit_box:
                break

        if not hit_box:
            break

        _, by1, _, by2 = hit_box
        above = by1 - 0.30
        below = by2 + 0.30
        corridor_y = above if abs(corridor_y - above) < abs(corridor_y - below) else below

    segs = []
    last_idx = len(pts) - 2
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 1e-4:
            continue

        if i == 0:
            w = WIDTH_015
        elif force_015_end and i == last_idx:
            w = WIDTH_015
        elif not allow_030:
            w = WIDTH_022
        else:
            w = WIDTH_022 if length < 1.4 else WIDTH_030

        segs.append(((x1, y1), (x2, y2), w))
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
        seg_block = text[start : end + 1]
        if '(net "/NRST")' in seg_block or '(net "NRST")' in seg_block:
            removed += 1
        else:
            out.append(seg_block)
        idx = end + 1
    return "".join(out), removed


def format_segments(seg_tuples, net_name: str, layer: str = "F.Cu") -> list[str]:
    lines = []
    for (x1, y1), (x2, y2), w in seg_tuples:
        lines.append(
            f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) '
            f'(width {w:.2f}) (layer "{layer}") (net "{net_name}"))'
        )
    return lines


def main() -> None:
    if not PCB_PATH.exists():
        raise FileNotFoundError(f"PCB file not found: {PCB_PATH}")

    text = PCB_PATH.read_text(encoding="utf-8")
    if paren_balance(text) != 0:
        raise RuntimeError("Parentheses unbalanced before editing.")

    fps = parse_footprints(text)
    fp_map = {fp["ref"]: fp for fp in fps}

    for ref, pin in [(SRC_REF, SRC_PIN), (C15_REF, C15_PIN), (C17_REF, C17_PIN), (CONN_REF, CONN_PIN)]:
        if ref not in fp_map or pin not in fp_map[ref]["pads"]:
            raise RuntimeError(f"Missing target pad {ref} pin {pin}.")

    p_u2_7 = fp_map[SRC_REF]["pads"][SRC_PIN]
    p_c15_2 = fp_map[C15_REF]["pads"][C15_PIN]
    p_c17_2 = fp_map[C17_REF]["pads"][C17_PIN]
    p_conn_5 = fp_map[CONN_REF]["pads"][CONN_PIN]

    print(f"U2.7={p_u2_7}, C15.2={p_c15_2}, C17.2={p_c17_2}, J2.5={p_conn_5}")

    segs_c15 = route_clear_corridor(
        p_u2_7,
        p_c15_2,
        initial_y=max(p_u2_7[1], p_c15_2[1]) + 1.5,
        fps=fps,
        ignore_refs={SRC_REF, C15_REF},
        allow_030=True,
        force_015_end=False,
    )

    segs_c17 = route_clear_corridor(
        p_u2_7,
        p_c17_2,
        initial_y=(p_u2_7[1] + p_c17_2[1]) * 0.5 + 1.2,
        fps=fps,
        ignore_refs={SRC_REF, C17_REF},
        allow_030=False,
        force_015_end=True,
    )

    segs_conn = route_clear_corridor(
        p_u2_7,
        p_conn_5,
        initial_y=(p_u2_7[1] + p_conn_5[1]) * 0.5 - 1.2,
        fps=fps,
        ignore_refs={SRC_REF, CONN_REF},
        allow_030=True,
        force_015_end=False,
    )

    all_segs = segs_c15 + segs_c17 + segs_conn
    cleaned, removed_cnt = remove_old_nrst_segments(text)
    block = "\n" + "\n".join(format_segments(all_segs, net_name=NET_NAME)) + "\n"

    pos = cleaned.rfind(")")
    if pos == -1:
        raise RuntimeError("Invalid .kicad_pcb format.")

    updated = cleaned[:pos] + block + cleaned[pos:]
    if paren_balance(updated) != 0:
        raise RuntimeError("Parentheses unbalanced after editing.")

    PCB_PATH.write_text(updated, encoding="utf-8")
    print(f"Removed old /NRST segments: {removed_cnt} | Added stepped segments: {len(all_segs)}")
    print(f"Parenthesis balance: {paren_balance(updated)}")


if __name__ == "__main__":
    main()
