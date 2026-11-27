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
BEYOND_COURTYARD_MM = 2.0
COURTYARD_FALLBACK_PAD = 0.50


def make_backup(pcb_path: Path) -> Path:
    BACKUP_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = BACKUP_DIR / f"{pcb_path.stem}_before_u2p7_2mm_{ts}.kicad_pcb"
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

    # Try extracting F.Courtyard rectangle bounds if present
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
        source = "F.Courtyard"
    else:
        min_x = min(pad_xs) - COURTYARD_FALLBACK_PAD
        max_x = max(pad_xs) + COURTYARD_FALLBACK_PAD
        min_y = min(pad_ys) - COURTYARD_FALLBACK_PAD
        max_y = max(pad_ys) + COURTYARD_FALLBACK_PAD
        source = "pads+margin"

    return pads[TARGET_PIN], (min_x, min_y, max_x, max_y), source


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


def main() -> None:
    if not PCB_PATH.exists():
        raise FileNotFoundError(f"PCB file not found: {PCB_PATH}")

    make_backup(PCB_PATH)
    text = PCB_PATH.read_text(encoding="utf-8")
    if paren_balance(text) != 0:
        raise RuntimeError("Unbalanced parentheses before editing.")

    u2_block = extract_footprint_block(text, TARGET_REF)
    (px, py), (min_x, min_y, max_x, max_y), crt_src = parse_u2_geometry(u2_block)

    # Choose the nearest horizontal courtyard exit side from pad X
    dist_left = abs(px - min_x)
    dist_right = abs(max_x - px)
    if dist_left <= dist_right:
        exit_x = min_x
        end_x = min_x - BEYOND_COURTYARD_MM
        side = "left"
    else:
        exit_x = max_x
        end_x = max_x + BEYOND_COURTYARD_MM
        side = "right"

    seg_line = (
        f'  (segment (start {px:.4f} {py:.4f}) (end {end_x:.4f} {py:.4f}) '
        f'(width {TRACK_WIDTH:.2f}) (layer "F.Cu") (net "{NET_NAME}"))'
    )

    cleaned, removed_cnt = remove_old_nrst_segments(text)
    end_pos = cleaned.rfind(")")
    if end_pos == -1:
        raise RuntimeError("Invalid .kicad_pcb file structure.")

    updated = cleaned[:end_pos] + "\n" + seg_line + "\n" + cleaned[end_pos:]
    if paren_balance(updated) != 0:
        raise RuntimeError("Unbalanced parentheses after editing.")

    PCB_PATH.write_text(updated, encoding="utf-8")
    print(f"U2 pin {TARGET_PIN}: ({px:.4f}, {py:.4f}) | Courtyard ({crt_src}) X=[{min_x:.4f}, {max_x:.4f}]")
    print(f"Exit side: {side} at x={exit_x:.4f} -> Endpoint at x={end_x:.4f} (+{BEYOND_COURTYARD_MM:.2f} mm beyond courtyard)")
    print(f"Removed old /NRST segments: {removed_cnt} | Width: {TRACK_WIDTH:.2f} mm | Balance: {paren_balance(updated)}")


if __name__ == "__main__":
    main()
