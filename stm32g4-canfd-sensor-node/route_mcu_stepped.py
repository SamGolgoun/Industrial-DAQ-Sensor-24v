import json
import math
import pathlib
import re
import subprocess

pcb_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
drc_path = pathlib.Path("/tmp/drc_mcu_stepped.json")
text = pcb_path.read_text(encoding="utf-8")


def find_matching_paren(src: str, open_idx: int) -> int:
depth = 0
in_str = False
esc = False
for i in range(open_idx, len(src)):
    ch = src[i]
    if esc:
        esc = False
    elif ch == "\\":
        esc = True
    elif ch == '"':
        in_str = not in_str
    elif not in_str:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return i
return -1


def iter_root_blocks(src: str, prefix: str):
root_end = find_matching_paren(src, 0)
if root_end < 0:
    return
i = 1
while i < root_end:
    if src[i] == "(":
        end_idx = find_matching_paren(src, i)
        if end_idx < 0:
            break
        blk = src[i : end_idx + 1]
        if blk.startswith(prefix):
            yield i, end_idx + 1, blk
        i = end_idx + 1
    else:
        i += 1


def parse_nets(src: str) -> dict[int, str]:
nets = {}
for _, _, blk in iter_root_blocks(src, "(net "):
    m = re.match(r'^\(\s*net\s+(\d+)\s+"([^"]*)"\s*\)$', blk.strip())
    if m:
        nets[int(m.group(1))] = m.group(2)
return nets


def parse_footprint(start: int, end: int, blk: str) -> dict:
ref_m = re.search(r'\(\s*property\s+"Reference"\s+"([^"]+)"', blk)
if not ref_m:
    ref_m = re.search(r'\(\s*fp_text\s+reference\s+"?([^"\s\)]+)', blk)
ref = ref_m.group(1).strip() if ref_m else ""

# Top-level footprint (at x y [deg])
fp_x, fp_y, fp_deg = 0.0, 0.0, 0.0
i = 1
depth = 1
in_str = False
esc = False
at_span = None
while i < len(blk):
    ch = blk[i]
    if esc:
        esc = False
    elif ch == "\\":
        esc = True
    elif ch == '"':
        in_str = not in_str
    elif not in_str:
        if ch == "(":
            if depth == 1 and re.match(r"^\(\s*at\b", blk[i : i + 10]):
                ae = find_matching_paren(blk, i)
                am = re.match(
                    r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$",
                    blk[i : ae + 1],
                )
                if am:
                    fp_x = float(am.group(1))
                    fp_y = float(am.group(2))
                    fp_deg = float(am.group(3)) if am.group(3) else 0.0
                    at_span = (i, ae + 1)
                    break
            depth += 1
        elif ch == ")":
            depth -= 1
    i += 1

pads = []
pi = 1
while pi < len(blk):
    if blk[pi] == "(" and blk[pi : pi + 5] == "(pad ":
        pe = find_matching_paren(blk, pi)
        if pe < 0:
            break
        pblk = blk[pi : pe + 1]
        pm = re.match(r'^\(\s*pad\s+"([^"]*)"\s+(\S+)\s+(\S+)', pblk)
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pblk)
        psm = re.search(r"\(\s*size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pblk)
        pnm = re.search(r'\(\s*net\s+(\d+)\s+"([^"]*)"', pblk)
        if pm and pam and psm:
            lx, ly = float(pam.group(1)), float(pam.group(2))
            sw, sh = float(psm.group(1)), float(psm.group(2))
            ncode = int(pnm.group(1)) if pnm else 0
            nname = pnm.group(2) if pnm else ""
            pads.append(
                {
                    "num": pm.group(1),
                    "type": pm.group(2),
                    "lx": lx,
                    "ly": ly,
                    "sw": sw,
                    "sh": sh,
                    "net_code": ncode,
                    "net_name": nname,
                }
            )
        pi = pe + 1
    else:
        pi += 1

return {
    "start": start,
    "end": end,
    "blk": blk,
    "ref": ref,
    "x": fp_x,
    "y": fp_y,
    "deg": fp_deg,
    "at_span": at_span,
    "pads": pads,
}


def pad_world_xy(fp: dict, pad: dict, cand_x: float = None, cand_y: float = None, cand_deg: float = None):
fx = fp["x"] if cand_x is None else cand_x
fy = fp["y"] if cand_y is None else cand_y
deg = fp["deg"] if cand_deg is None else cand_deg
# KiCad footprint rotation convention
rad = math.radians(-deg)
wx = fx + pad["lx"] * math.cos(rad) - pad["ly"] * math.sin(rad)
wy = fy + pad["lx"] * math.sin(rad) + pad["ly"] * math.cos(rad)
return wx, wy


def segments_intersect(a1, a2, b1, b2) -> bool:
def ccw(p1, p2, p3):
    return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])

if a1 == b1 or a1 == b2 or a2 == b1 or a2 == b2:
    return False
return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))


def pt_seg_dist(px, py, x1, y1, x2, y2) -> float:
dx, dy = x2 - x1, y2 - y1
l2 = dx * dx + dy * dy
if l2 == 0.0:
    return math.hypot(px - x1, py - y1)
t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def seg_seg_dist(a1, a2, b1, b2) -> float:
if segments_intersect(a1, a2, b1, b2):
    return 0.0
return min(
    pt_seg_dist(a1[0], a1[1], b1[0], b1[1], b2[0], b2[1]),
    pt_seg_dist(a2[0], a2[1], b1[0], b1[1], b2[0], b2[1]),
    pt_seg_dist(b1[0], b1[1], a1[0], a1[1], a2[0], a2[1]),
    pt_seg_dist(b2[0], b2[1], a1[0], a1[1], a2[0], a2[1]),
)


def main():
global text
fps = [parse_footprint(s, e, b) for s, e, b in iter_root_blocks(text, "(footprint ")]
by_ref = {fp["ref"]: fp for fp in fps}

u1 = by_ref.get("U1")
if not u1:
    print("U1 not found in board.")
    return

u1_nets = {p["net_name"] for p in u1["pads"] if p["net_name"] and p["net_name"] != "GND"}
print(f"U1 center=({u1['x']:.3f}, {u1['y']:.3f}), signal/power nets={len(u1_nets)}")

# 1. Optimize orientation (0/90/180/270) and micro-nudges for 2-pin passives/diodes/crystal connected to U1
movable_prefixes = ("C", "R", "D", "Y", "FB")
opt_logs = []

def net_wirelength_and_crossings(target_fp, cx, cy, cdeg):
    # Compute distance from target_fp pads to their nearest matching pad on U1 or GND via/pad
    dist_sum = 0.0
    for p in target_fp["pads"]:
        if not p["net_name"]:
            continue
        wx, wy = pad_world_xy(target_fp, p, cx, cy, cdeg)
        if p["net_name"] == "GND":
            # Prefer GND pad facing outward/toward nearest GND reference or board interior
            best = min(
                math.hypot(wx - op_xy[0], wy - op_xy[1])
                for ofp in fps
                if ofp["ref"] != target_fp["ref"]
                for op in ofp["pads"]
                if op["net_name"] == "GND"
                for op_xy in [pad_world_xy(ofp, op)]
            )
            dist_sum += 0.6 * best
        else:
            matches = [
                pad_world_xy(u1, up)
                for up in u1["pads"]
                if up["net_name"] == p["net_name"]
            ]
            if matches:
                dist_sum += min(math.hypot(wx - ux, wy - uy) for ux, uy in matches)
    # Penalize internal wire crossing between the 2 pads of target_fp when connecting to U1
    if len(target_fp["pads"]) == 2:
        p0, p1 = target_fp["pads"][0], target_fp["pads"][1]
        w0 = pad_world_xy(target_fp, p0, cx, cy, cdeg)
        w1 = pad_world_xy(target_fp, p1, cx, cy, cdeg)
        m0 = [pad_world_xy(u1, up) for up in u1["pads"] if up["net_name"] == p0["net_name"]]
        m1 = [pad_world_xy(u1, up) for up in u1["pads"] if up["net_name"] == p1["net_name"]]
        if m0 and m1:
            t0 = min(m0, key=lambda q: math.hypot(w0[0] - q[0], w0[1] - q[1]))
            t1 = min(m1, key=lambda q: math.hypot(w1[0] - q[0], w1[1] - q[1]))
            if segments_intersect(w0, t0, w1, t1):
                dist_sum += 6.0
    return dist_sum

def clear_of_other_pads(target_fp, cx, cy, cdeg) -> bool:
    for p in target_fp["pads"]:
        wx, wy = pad_world_xy(target_fp, p, cx, cy, cdeg)
        r1 = max(p["sw"], p["sh"]) * 0.5 + 0.35
        for ofp in fps:
            if ofp["ref"] == target_fp["ref"]:
                continue
            for op in ofp["pads"]:
                ox, oy = pad_world_xy(ofp, op)
                r2 = max(op["sw"], op["sh"]) * 0.5 + 0.35
                if math.hypot(wx - ox, wy - oy) < (r1 + r2):
                    return False
    return True

for fp in fps:
    ref = fp["ref"]
    if not any(ref.startswith(pref) for pref in movable_prefixes):
        continue
    fp_nets = {p["net_name"] for p in fp["pads"] if p["net_name"]}
    if not (fp_nets & u1_nets):
        continue
    if len(fp["pads"]) > 4:
        continue

    base_score = net_wirelength_and_crossings(fp, fp["x"], fp["y"], fp["deg"])
    best_cfg = (fp["x"], fp["y"], fp["deg"], base_score)

    # Evaluate 90-degree rotations and small grid nudges (-0.8, -0.4, 0, +0.4, +0.8 mm)
    cand_degs = sorted({fp["deg"] % 360.0, 0.0, 90.0, 180.0, 270.0})
    nudges = [0.0, -0.4, 0.4, -0.8, 0.8]
    for deg in cand_degs:
        for dx in nudges:
            for dy in nudges:
                cx = round(fp["x"] + dx, 3)
                cy = round(fp["y"] + dy, 3)
                if not (104.0 <= cx <= 176.0 and 64.0 <= cy <= 116.0):
                    continue
                if not clear_of_other_pads(fp, cx, cy, deg):
                    continue
                sc = net_wirelength_and_crossings(fp, cx, cy, deg)
                if sc + 0.15 < best_cfg[3]:
                    best_cfg = (cx, cy, deg, sc)

    if (best_cfg[0], best_cfg[1], best_cfg[2]) != (fp["x"], fp["y"], fp["deg"]):
        opt_logs.append(
            (ref, (fp["x"], fp["y"], fp["deg"]), (best_cfg[0], best_cfg[1], best_cfg[2]), round(base_score - best_cfg[3], 2))
        )
        fp["x"], fp["y"], fp["deg"] = best_cfg[0], best_cfg[1], best_cfg[2]

# Write updated footprint positions/rotations back into PCB S-expression
repls = []
for fp in fps:
    if fp["at_span"]:
        s_at, e_at = fp["at_span"]
        new_at = f"(at {fp['x']:.3f} {fp['y']:.3f} {fp['deg']:.0f})" if fp["deg"] != 0 else f"(at {fp['x']:.3f} {fp['y']:.3f})"
        new_blk = fp["blk"][:s_at] + new_at + fp["blk"][e_at:]
        repls.append((fp["start"], fp["end"], new_blk))

for s, e, nb in sorted(repls, key=lambda item: item[0], reverse=True):
    text = text[:s] + nb + text[e:]

print(f"\n=== COMPONENT ORIENTATION & PLACEMENT OPTIMIZATION ({len(opt_logs)} adjusted) ===")
for ref, old_c, new_c, gain in opt_logs:
    print(f"  {ref}: ({old_c[0]:.2f}, {old_c[1]:.2f}, {old_c[2]:.0f}deg) -> ({new_c[0]:.2f}, {new_c[1]:.2f}, {new_c[2]:.0f}deg) | wirelength gain={gain} mm")

# 2. Stepped Routing for U1 signals and analog/power branches on F.Cu (no acute <90 deg angles)
# Build pad obstacle list
all_pads = []
for fp in fps:
    for p in fp["pads"]:
        wx, wy = pad_world_xy(fp, p)
        all_pads.append(
            {
                "ref": fp["ref"],
                "num": p["num"],
                "x": wx,
                "y": wy,
                "r": max(p["sw"], p["sh"]) * 0.5,
                "net": p["net_name"],
                "code": p["net_code"],
            }
        )

routed_segs = []  # (x1, y1, x2, y2, width, layer, net_code, net_name)

def segment_is_clear(x1, y1, x2, y2, width, net_name) -> bool:
    # Check clearance against all pads of different nets
    for pd in all_pads:
        if pd["net"] == net_name:
            continue
        d = pt_seg_dist(pd["x"], pd["y"], x1, y1, x2, y2)
        if d < (pd["r"] + width * 0.5 + 0.16):
            return False
    # Check clearance against already routed segments of different nets
    for sx1, sy1, sx2, sy2, sw, _, _, snet in routed_segs:
        if snet == net_name:
            continue
        if seg_seg_dist((x1, y1), (x2, y2), (sx1, sy1), (sx2, sy2)) < (width * 0.5 + sw * 0.5 + 0.16):
            return False
    return True

def build_stepped_45deg_path(p_start, p_end):
    """
    Build a stepped trace with >=135 deg obtuse bends (no <90 deg acute angles):
    - 0.15 mm neck-down exiting U1 pin
    - 0.25..0.30 mm main trunk in open space (0.35..0.40 mm for power/resistor/diode target)
    - 0.20..0.30 mm entry into target pad
    """
    x1, y1 = p_start["x"], p_start["y"]
    x2, y2 = p_end["x"], p_end["y"]
    net_name = p_start["net"]
    net_code = p_start["code"]
    dist = math.hypot(x2 - x1, y2 - y1)
    if dist < 0.25:
        return []

    # Determine escape direction away from U1 center
    vx = x1 - u1["x"]
    vy = y1 - u1["y"]
    if abs(vx) >= abs(vy):
        esc_dx = 1.0 if vx >= 0 else -1.0
        esc_dy = 0.0
    else:
        esc_dx = 0.0
        esc_dy = 1.0 if vy >= 0 else -1.0

    neck_len = min(1.1, max(0.55, dist * 0.22))
    nx1 = round(x1 + esc_dx * neck_len, 4)
    ny1 = round(y1 + esc_dy * neck_len, 4)

    # Target entry width: wider (0.35-0.40 mm) if target is R/D/C/power with wide pad, else 0.25-0.30 mm
    is_wide_target = p_end["ref"].startswith(("R", "D", "L", "FB", "F")) or ("3V3" in net_name)
    trunk_w = 0.35 if is_wide_target else 0.25
    entry_w = 0.40 if (is_wide_target and p_end["r"] >= 0.35) else 0.25

    # Construct obtuse 135-deg chamfered dogleg from (nx1, ny1) to (x2, y2)
    dx = x2 - nx1
    dy = y2 - ny1
    adx, ady = abs(dx), abs(dy)
    diag = min(adx, ady)
    sx = 1.0 if dx >= 0 else -1.0
    sy = 1.0 if dy >= 0 else -1.0

    # Two candidate chamfer orders (diagonal first vs straight first) - both have 135 deg internal corners
    mx_a = round(nx1 + sx * diag, 4)
    my_a = round(ny1 + sy * diag, 4)

    if adx >= ady:
        mx_b = round(nx1 + sx * (adx - diag), 4)
        my_b = round(ny1, 4)
    else:
        mx_b = round(nx1, 4)
        my_b = round(ny1 + sy * (ady - diag), 4)

    candidates = [
        [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, mx_a, my_a, trunk_w), (mx_a, my_a, x2, y2, entry_w)],
        [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, mx_b, my_b, trunk_w), (mx_b, my_b, x2, y2, entry_w)],
        [(x1, y1, nx1, ny1, 0.15), (nx1, ny1, x2, y2, trunk_w)],
    ]

    for cand in candidates:
        clean_cand = [(round(a, 4), round(b, 4), round(c, 4), round(d, 4), w) for a, b, c, d, w in cand if math.hypot(c - a, d - b) > 0.03]
        if clean_cand and all(segment_is_clear(a, b, c, d, w, net_name) for a, b, c, d, w in clean_cand):
            return [(a, b, c, d, w, "F.Cu", net_code, net_name) for a, b, c, d, w in clean_cand]
    return []

# Prioritize sensitive nets first: OSC, ADC, +3V3_ANA, then remaining U1 signals
def net_priority(n: str) -> int:
    nu = n.upper()
    if "OSC" in nu:
        return 0
    if "ADC" in nu or "ANA" in nu:
        return 1
    if "CAN" in nu or "SPI" in nu or "UART" in nu:
        return 2
    return 3

u1_sig_pads = sorted(
    [pd for pd in all_pads if pd["ref"] == "U1" and pd["net"] and pd["net"] != "GND"],
    key=lambda item: (net_priority(item["net"]), item["num"]),
)

routed_nets_summary = []
for up in u1_sig_pads:
    targets = [
        pd
        for pd in all_pads
        if pd["net"] == up["net"] and not (pd["ref"] == "U1" and pd["num"] == up["num"])
    ]
    if not targets:
        continue
    # Connect U1 pin to its nearest partner pad
    nearest = min(targets, key=lambda tp: math.hypot(tp["x"] - up["x"], tp["y"] - up["y"]))
    segs = build_stepped_45deg_path(up, nearest)
    if segs:
        routed_segs.extend(segs)
        widths = [s[4] for s in segs]
        routed_nets_summary.append((up["net"], f"U1.{up['num']}->{nearest['ref']}.{nearest['num']}", widths))

# Append generated stepped segments before final closing paren of PCB file
if routed_segs:
    seg_lines = []
    for x1, y1, x2, y2, w, layer, ncode, _ in routed_segs:
        seg_lines.append(
            f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width {w:.2f}) (layer "{layer}") (net {ncode}))'
        )
    last_paren = text.rfind(")")
    text = text[:last_paren] + "\n" + "\n".join(seg_lines) + "\n)\n"

pcb_path.write_text(text, encoding="utf-8")

print(f"\n=== STEPPED MCU ROUTING SUMMARY ({len(routed_nets_summary)} nets, {len(routed_segs)} segments) ===")
for net_n, conn, ws in routed_nets_summary:
    print(f"  {net_n:18s} | {conn:16s} | widths(mm)={ws}")

# Run full DRC and print violations list
subprocess.run(
    [
        "/tmp/kcli.sh",
        "pcb",
        "drc",
        "--refill-zones",
        "--save-board",
        "--severity-all",
        "--format",
        "json",
        "--output",
        str(drc_path),
        str(pcb_path),
    ],
    check=True,
)

rep = json.loads(drc_path.read_text(encoding="utf-8"))
viols = rep.get("violations", [])
unconn = rep.get("unconnected_items", [])
print(f"\n=== DRC REPORT: {len(viols)} VIOLATIONS | {len(unconn)} UNCONNECTED ===")
for idx, v in enumerate(viols, 1):
    print(f"  {idx}. [{v.get('severity', 'error')}] ({v.get('type', '?')}) {v.get('description', '')}")


if __name__ == "__main__":
main()
