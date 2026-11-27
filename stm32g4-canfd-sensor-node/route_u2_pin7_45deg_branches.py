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

STUB_WIDTH = 0.15
BRANCH_WIDTH = 0.15
BEYOND_COURTYARD_MM = 1.0
BRANCH_LEN_MM = 3.0
COURTYARD_FALLBACK_PAD = 0.50


def make_backup(pcb_path: Path) -> Path:
    BACKUP_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BACKUP_DIR / f"{pcb_path.stem}_before_45deg_branches_{ts}.kicad_pcb"
    shutil.copy2(pcb_path, out)
    if not out.exists() or out.stat().st_size == 0:
        raise RuntimeError(f"Backup failed: {out}")
    print(f"[BACKUP] {out}")
    return out


def paren_balance(text: str) -> int:
    return text.count("(") - text.count(")")


def extract_footprint_block(text: str, ref: str) -> str:
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
        if re.search(rf'\(property\s+"Reference"\s+"{re.escape(ref)}"', block):
            return block
        idx = end + 1
    raise RuntimeError(f"Footprint '{ref}' not found.")


def parse_u2_geometry(fp_block: str):
    at_m = re.search(r"\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)", fp_block)
    fx = float(at_m.group(1)) if at_m else 0.0
    fy = float(at_m.group(2)) if at_m else 0.0
    rot = math.radians(float(at_m.group(3))) if (at_m and at_m.group(3)) else 0.0

    pads = {}
    pad_xs, pad_ys = [], []
    for pm in re.finditer(
        r'\(pad\s+"([^"]+)"\s+\S+\s+\S+\s+\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+[-\d.]+)?\)',
        fp_block,
    ):
        pin = pm.group(1)
        lx = float(pm.group(2))
        ly = float(pm.group(3))
        gx = fx + lx * math.cos(rot) - ly * math.sin(rot)
        gy = fy + lx * math.sin(rot) + ly * math.cos(rot)
        pads[pin] = (gx, gy)
        pad_xs.append(gx)
        pad_ys.append(gy)

    if TARGET_PIN not in pads:
        raise RuntimeError(f"Pin '{TARGET_PIN}' not found on '{TARGET_REF}'.")

    crt_xs, crt_ys = [], []
    for line in fp_block.splitlines():
        if '(layer "F.Courtyard")' in line:
            for pt_m in re.finditer(r"\((?:xy|start|end)\s+([-\d.]+)\s+([-\d.]+)\)", line):
                lx = float(pt_m.group(1))
                ly = float(pt_m.group(2))
                crt_xs.append(fx + lx * math.cos(rot) - ly * math.sin(rot))
                crt_ys.append(fy + lx * math.sin(rot) + ly * math.cos(rot))

    if crt_xs and crt_ys:
        min_x, max_x = min(crt_xs), max(crt_xs)
        min_y, max_y = min(crt_ys), max(crt_ys)
    else:
        min_x = min(pad_xs) - COURTYARD_FALLBACK_PAD
        max_x = max(pad_xs) + COURTYARD_FALLBACK_PAD
        min_y = min(pad_ys) - COURTYARD_FALLBACK_PAD
        max_y = max(pad_ys) + COURTYARD_FALLBACK_PAD

    return pads[TARGET_PIN], (min_x, min_y, max_x, max_y)


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

    u2_block = extract_footprint_block(text, TARGET_REF)
    (px, py), (min_x, _, max_x, _) = parse_u2_geometry(u2_block)

    if abs(px - min_x) <= abs(max_x - px):
        hub_x = min_x - BEYOND_COURTYARD_MM
        dir_x = -1.0
    else:
        hub_x = max_x + BEYOND_COURTYARD_MM
        dir_x = 1.0
    hub_y = py

    # In KiCad board coordinates, +Y is downward and -Y is upward
    rad45 = math.radians(45.0)
    up_x = hub_x + dir_x * BRANCH_LEN_MM * math.cos(rad45)
    up_y = hub_y - BRANCH_LEN_MM * math.sin(rad45)

    dn_x = hub_x + dir_x * BRANCH_LEN_MM * math.cos(rad45)
    dn_y = hub_y + BRANCH_LEN_MM * math.sin(rad45)

    # Verify angle between the two 3 mm branches from hub
    v_up = (up_x - hub_x, up_y - hub_y)
    v_dn = (dn_x - hub_x, dn_y - hub_y)
    dot = v_up[0] * v_dn[0] + v_up[1] * v_dn[1]
    n_up = math.hypot(*v_up)
    n_dn = math.hypot(*v_dn)
    between_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot / (n_up * n_dn)))))

    segments = [
        (px, py, hub_x, hub_y, STUB_WIDTH, "stub_1mm"),
        (hub_x, hub_y, up_x, up_y, BRANCH_WIDTH, "branch_up_45deg_3mm"),
        (hub_x, hub_y, dn_x, dn_y, BRANCH_WIDTH, "branch_down_45deg_3mm"),
    ]

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
    print(f"Angle between up and down 3.0 mm branches: {between_deg:.1f} deg")
    for idx, (x1, y1, x2, y2, w, label) in enumerate(segments, 1):
        length = math.hypot(x2 - x1, y2 - y1)
        print(f"  #{idx} [{label}]: ({x1:.4f}, {y1:.4f}) -> ({x2:.4f}, {y2:.4f}) | len={length:.4f} mm | w={w:.2f} mm")


if __name__ == "__main__":
    main()
