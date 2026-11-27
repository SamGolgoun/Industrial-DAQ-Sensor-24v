import math
import re
import shutil
from datetime import datetime
from pathlib import Path

PCB_PATH = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BACKUP_DIR = Path("backups")
NET_NAME = "/NRST"
TARGET_REF = "U2"
TARGET_PIN = "7"

TRACK_WIDTH = 0.15
BEYOND_COURTYARD_MM = 1.0
BRANCH_LEN_MM = 1.0
MIN_CLEARANCE_MM = 0.25
COURTYARD_FALLBACK_PAD = 0.50


def make_backup(pcb_path: Path) -> Path:
    BACKUP_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BACKUP_DIR / f"{pcb_path.stem}_before_u2p7_1mm_fix_{ts}.kicad_pcb"
    shutil.copy2(pcb_path, out)
    if not out.exists() or out.stat().st_size == 0:
        raise RuntimeError(f"Backup failed: {out}")
    print(f"[BACKUP] {out}")
    return out


def paren_balance(text: str) -> int:
    return text.count("(") - text.count(")")


def parse_all_footprints(text: str):
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
        pad_xs, pad_ys = [], []
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
            pad_xs.append(gx)
            pad_ys.append(gy)

        crt_xs, crt_ys = [], []
        for line in block.splitlines():
            if '(layer "F.Courtyard")' in line:
                for pt_m in re.finditer(r"\((?:xy|start|end)\s+([-\d.]+)\s+([-\d.]+)\)", line):
                    lx = float(pt_m.group(1))
                    ly = float(pt_m.group(2))
                    crt_xs.append(fx + lx * math.cos(rot) - ly * math.sin(rot))
                    crt_ys.append(fy + lx * math.sin(rot) + ly * math.cos(rot))

        if crt_xs and crt_ys:
            box = (min(crt_xs), min(crt_ys), max(crt_xs), max(crt_ys))
        elif pad_xs and pad_ys:
            box = (
                min(pad_xs) - COURTYARD_FALLBACK_PAD,
                min(pad_ys) - COURTYARD_FALLBACK_PAD,
                max(pad_xs) + COURTYARD_FALLBACK_PAD,
                max(pad_ys) + COURTYARD_FALLBACK_PAD,
            )
        else:
            box = (fx - 1.0, fy - 1.0, fx + 1.0, fy + 1.0)

        fps.append({"ref": ref, "pads": pads, "box": box})
    return fps


def seg_box_distance(p1, p2, box) -> float:
    min_x, min_y, max_x, max_y = box
    best = float("inf")
    for k in range(41):
        t = k / 40.0
        px = p1[0] + t * (p2[0] - p1[0])
        py = p1[1] + t * (p2[1] - p1[1])
        dx = max(min_x - px, 0.0, px - max_x)
        dy = max(min_y - py, 0.0, py - max_y)
        d = math.hypot(dx, dy)
        if d < best:
            best = d
    return best


def seg_to_seg_non_hub_distance(a1, a2, b1, b2) -> float:
    # Sample along A excluding points within 0.05 mm of shared hub (b1)
    best = float("inf")
    bx, by = b2[0] - b1[0], b2[1] - b1[1]
    b_len2 = bx * bx + by * by
    for k in range(1, 41):
        t = k / 40.0
        px = a1[0] + t * (a2[0] - a1[0])
        py = a1[1] + t * (a2[1] - a1[1])
        if math.hypot(px - b1[0], py - b1[1]) < 0.05:
            continue
        if b_len2 < 1e-12:
            d = math.hypot(px - b1[0], py - b1[1])
        else:
            u = max(0.0, min(1.0, ((px - b1[0]) * bx + (py - b1[1]) * by) / b_len2))
            qx = b1[0] + u * bx
            qy = b1[1] + u * by
            d = math.hypot(px - qx, py - qy)
        if d < best:
            best = d
    return best


def run_rule_gate(segments, fps, src_ref):
    violations = []

    for idx, (x1, y1, x2, y2, w, label) in enumerate(segments, 1):
        if abs(w - TRACK_WIDTH) > 1e-6:
            violations.append(f"Segment #{idx} [{label}] width {w:.2f} != {TRACK_WIDTH:.2f} mm")

    up_seg = next((s for s in segments if s[5] == "branch_up_1mm"), None)
    dn_seg = next((s for s in segments if s[5] == "branch_down_1mm"), None)
    if up_seg and dn_seg:
        v1 = (up_seg[2] - up_seg[0], up_seg[3] - up_seg[1])
        v2 = (dn_seg[2] - dn_seg[0], dn_seg[3] - dn_seg[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 > 1e-6 and n2 > 1e-6:
            cos_a = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
            ang = math.degrees(math.acos(cos_a))
            if abs(ang - 90.0) > 0.5:
                violations.append(f"Angle between up/down branches is {ang:.1f} deg (expected 90.0 deg)")

    for idx, (x1, y1, x2, y2, w, label) in enumerate(segments, 1):
        p1, p2 = (x1, y1), (x2, y2)
        for fp in fps:
            if fp["ref"] == src_ref:
                continue
            dist = seg_box_distance(p1, p2, fp["box"])
            required = MIN_CLEARANCE_MM + w * 0.5
            if dist < required:
                violations.append(
                    f"Segment #{idx} [{label}] ({x1:.3f},{y1:.3f})->({x2:.3f},{y2:.3f}) "
                    f"violates clearance to {fp['ref']} courtyard: dist={dist:.3f} mm < {required:.3f} mm"
                )

    # Check clearance between the two 1.0 mm branches away from their shared hub
    if up_seg and dn_seg:
        d = seg_to_seg_non_hub_distance(
            (up_seg[0], up_seg[1]), (up_seg[2], up_seg[3]),
            (dn_seg[0], dn_seg[1]), (dn_seg[2], dn_seg[3]),
        )
        if d < MIN_CLEARANCE_MM + TRACK_WIDTH:
            violations.append(f"Branches [branch_up_1mm] and [branch_down_1mm] too close: dist={d:.3f} mm")

    return violations


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


def fmt_seg(x1, y1, x2, y2, width, net_name):
    return (
        f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) '
        f'(width {width:.2f}) (layer "F.Cu") (net "{net_name}"))'
    )


def main() -> None:
    if not PCB_PATH.exists():
        raise FileNotFoundError(f"PCB file not found: {PCB_PATH}")

    make_backup(PCB_PATH)
    text = PCB_PATH.read_text(encoding="utf-8")
    if paren_balance(text) != 0:
        raise RuntimeError("Unbalanced parentheses before editing.")

    fps = parse_all_footprints(text)
    u2_fp = next((fp for fp in fps if fp["ref"] == TARGET_REF), None)
    if not u2_fp or TARGET_PIN not in u2_fp["pads"]:
        raise RuntimeError(f"Footprint '{TARGET_REF}' or pin '{TARGET_PIN}' not found.")

    px, py = u2_fp["pads"][TARGET_PIN]
    min_x, _, max_x, _ = u2_fp["box"]

    if abs(px - min_x) <= abs(max_x - px):
        hub_x = min_x - BEYOND_COURTYARD_MM
        dir_x = -1.0
    else:
        hub_x = max_x + BEYOND_COURTYARD_MM
        dir_x = 1.0
    hub_y = py

    rad45 = math.radians(45.0)
    up_x = hub_x + dir_x * BRANCH_LEN_MM * math.cos(rad45)
    up_y = hub_y - BRANCH_LEN_MM * math.sin(rad45)

    dn_x = hub_x + dir_x * BRANCH_LEN_MM * math.cos(rad45)
    dn_y = hub_y + BRANCH_LEN_MM * math.sin(rad45)

    segments = [
        (px, py, hub_x, hub_y, TRACK_WIDTH, "stub_1mm"),
        (hub_x, hub_y, up_x, up_y, TRACK_WIDTH, "branch_up_1mm"),
        (hub_x, hub_y, dn_x, dn_y, TRACK_WIDTH, "branch_down_1mm"),
    ]

    violations = run_rule_gate(segments, fps, TARGET_REF)
    if violations:
        print("RULE_GATE_FAILED: Aborting write! No tracks were drawn or modified on PCB.")
        for v in violations:
            print("  [RULE VIOLATION]", v)
        raise SystemExit(1)

    print("RULE_GATE_PASSED: All design rules and courtyard clearances verified.")

    cleaned, removed_cnt = remove_old_nrst_segments(text)
    block = "\n" + "\n".join(fmt_seg(x1, y1, x2, y2, w, NET_NAME) for x1, y1, x2, y2, w, _ in segments) + "\n"

    end_pos = cleaned.rfind(")")
    if end_pos == -1:
        raise RuntimeError("Invalid .kicad_pcb file structure.")

    updated = cleaned[:end_pos] + block + cleaned[end_pos:]
    if paren_balance(updated) != 0:
        raise RuntimeError("Unbalanced parentheses after editing.")

    PCB_PATH.write_text(updated, encoding="utf-8")
    print(f"Removed old /NRST segments: {removed_cnt} | Balance: {paren_balance(updated)}")
    for idx, (x1, y1, x2, y2, w, label) in enumerate(segments, 1):
        length = math.hypot(x2 - x1, y2 - y1)
        print(f"  #{idx} [{label}]: ({x1:.4f}, {y1:.4f}) -> ({x2:.4f}, {y2:.4f}) | len={length:.4f} mm | w={w:.2f} mm")


if __name__ == "__main__":
    main()
