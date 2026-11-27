#!/usr/bin/env python3
"""Rotate footprints to improve estimated ratsnest routing and refresh pad colors."""

import argparse
import datetime
import hashlib
import html
import json
import math
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import pcbnew


ROTATIONS = (0, 90, 180, 270)
FIXED_REFERENCES = {"H1", "H2", "H3", "H4", "FID1", "FID2", "FID3"}
COURTYARD_CLEARANCE_MM = 0.15
OBSTACLE_MARGIN_MM = 0.30
CROSSING_WEIGHT_MM = 5.0
BLOCKED_LENGTH_WEIGHT = 0.25
MAX_PASSES = 3
MAX_SAFE_PASSES = 2
MAX_DRC_TRIALS = 48


def check_safe_to_write(board_path: Path) -> None:
    if list(board_path.parent.glob("~*.kicad_pcb.lck")):
        raise RuntimeError("PCB lock present; close KiCad before continuing")
    result = subprocess.run(
        ["ps", "-eo", "comm="], capture_output=True, text=True, check=True
    )
    if any(line.strip() == "pcbnew" for line in result.stdout.splitlines()):
        raise RuntimeError("pcbnew is running; close it before continuing")


def balanced_sexpression(text: str) -> bool:
    depth = 0
    quoted = False
    escaped = False
    comment = False
    for char in text:
        if comment:
            if char == "\n":
                comment = False
        elif quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        elif char == ";":
            comment = True
        elif char == '"':
            quoted = True
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth < 0:
                return False
    return depth == 0 and not quoted


def matching_paren(text: str, open_index: int) -> int:
    depth = 0
    quoted = False
    escaped = False
    comment = False
    for index in range(open_index, len(text)):
        char = text[index]
        if comment:
            if char == "\n":
                comment = False
        elif quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        elif char == ";":
            comment = True
        elif char == '"':
            quoted = True
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return index
    return -1


def root_blocks(text: str, prefix: str):
    root_start = text.find("(")
    if root_start < 0:
        raise ValueError("PCB S-expression is empty")
    root_end = matching_paren(text, root_start)
    if root_end < 0:
        raise ValueError("PCB S-expression is unbalanced")
    index = root_start + 1
    while index < root_end:
        if text[index].isspace() or text[index] == ";":
            index += 1
            continue
        if text[index] != "(":
            index += 1
            continue
        end = matching_paren(text, index)
        if end < 0:
            raise ValueError("PCB child S-expression is unbalanced")
        block = text[index : end + 1]
        if block.startswith(prefix):
            yield index, end + 1, block
        index = end + 1


def footprint_reference(block: str) -> str:
    match = re.search(r'\(\s*property\s+"Reference"\s+"([^"]+)"', block)
    if not match:
        match = re.search(r'\(\s*fp_text\s+reference\s+"?([^"\s\)]+)', block)
    return match.group(1).upper() if match else ""


def patch_footprint_angles(text: str, desired_angles: dict[str, int]) -> str:
    replacements = []
    found = set()
    for start, end, block in root_blocks(text, "(footprint "):
        reference = footprint_reference(block)
        if reference not in desired_angles:
            continue
        depth = 1
        quoted = False
        escaped = False
        index = 1
        while index < len(block):
            char = block[index]
            if quoted:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    quoted = False
            elif char == '"':
                quoted = True
            elif char == "(":
                if depth == 1 and re.match(r"^\(\s*at\b", block[index : index + 10]):
                    at_end = matching_paren(block, index)
                    old_at = block[index : at_end + 1]
                    match = re.match(
                        r'^(\(\s*at\s+-?\d+(?:\.\d+)?\s+-?\d+(?:\.\d+)?)'
                        r"(?:\s+-?\d+(?:\.\d+)?)?(\s*\))$",
                        old_at,
                    )
                    if not match:
                        raise ValueError(f"Could not patch placement for {reference}")
                    new_at = f"{match.group(1)} {desired_angles[reference]}{match.group(2)}"
                    replacements.append((start, end, block[:index] + new_at + block[at_end + 1 :]))
                    found.add(reference)
                    break
                depth += 1
            elif char == ")":
                depth -= 1
            index += 1
        else:
            raise ValueError(f"Placement not found for {reference}")
    if found != set(desired_angles):
        raise ValueError(f"Could not find footprint references: {sorted(set(desired_angles) - found)}")
    for start, end, replacement in sorted(replacements, reverse=True):
        text = text[:start] + replacement + text[end:]
    return text


def find_cli(explicit: str | None) -> str:
    if explicit:
        return explicit
    found = shutil.which("kicad-cli")
    if found:
        return found
    candidates = sorted(Path("/tmp").glob(".mount_kicad*/usr/bin/kicad-cli"))
    if not candidates:
        raise FileNotFoundError("kicad-cli was not found")
    return str(candidates[-1])


def create_backup(board_path: Path) -> Path:
    backup_dir = board_path.parent / "backups"
    backup_dir.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%f%z")
    backup_path = backup_dir / f"{board_path.stem}_{stamp}_before_rotation.kicad_pcb"
    shutil.copy2(board_path, backup_path)
    if hashlib.sha256(backup_path.read_bytes()).digest() != hashlib.sha256(
        board_path.read_bytes()
    ).digest():
        raise IOError("Timestamped backup verification failed")
    return backup_path


def footprint_box(footprint):
    courtyard = (
        pcbnew.B_CrtYd
        if footprint.GetLayer() == pcbnew.B_Cu
        else pcbnew.F_CrtYd
    )
    boxes = [
        item.GetBoundingBox()
        for item in footprint.GraphicalItems()
        if item.GetLayer() == courtyard
    ]
    if not boxes:
        box = footprint.GetBoundingBox(False, False)
        return (
            box.GetLeft() / 1e6,
            box.GetTop() / 1e6,
            box.GetRight() / 1e6,
            box.GetBottom() / 1e6,
        )
    return (
        min(box.GetLeft() for box in boxes) / 1e6,
        min(box.GetTop() for box in boxes) / 1e6,
        max(box.GetRight() for box in boxes) / 1e6,
        max(box.GetBottom() for box in boxes) / 1e6,
    )


def boxes_overlap(a, b, clearance: float = COURTYARD_CLEARANCE_MM) -> bool:
    return (
        a[0] < b[2] + clearance
        and a[2] > b[0] - clearance
        and a[1] < b[3] + clearance
        and a[3] > b[1] - clearance
    )


def courtyard_overlaps(footprints) -> set[tuple[str, str]]:
    entries = [
        (fp.GetReference(), fp.GetLayer() == pcbnew.B_Cu, footprint_box(fp))
        for fp in footprints
    ]
    overlaps = set()
    for index, (ref_a, back_a, box_a) in enumerate(entries):
        for ref_b, back_b, box_b in entries[index + 1 :]:
            if back_a == back_b and boxes_overlap(box_a, box_b):
                overlaps.add(tuple(sorted((ref_a, ref_b))))
    return overlaps


def outside_board(footprints, board_box) -> set[str]:
    bounds = (
        board_box.GetLeft() / 1e6,
        board_box.GetTop() / 1e6,
        board_box.GetRight() / 1e6,
        board_box.GetBottom() / 1e6,
    )
    result = set()
    for footprint in footprints:
        box = footprint_box(footprint)
        if (
            box[0] < bounds[0] + COURTYARD_CLEARANCE_MM
            or box[1] < bounds[1] + COURTYARD_CLEARANCE_MM
            or box[2] > bounds[2] - COURTYARD_CLEARANCE_MM
            or box[3] > bounds[3] - COURTYARD_CLEARANCE_MM
        ):
            result.add(footprint.GetReference())
    return result


def segment_intersects_box(start, end, box, margin: float) -> bool:
    left, top, right, bottom = (
        box[0] - margin,
        box[1] - margin,
        box[2] + margin,
        box[3] + margin,
    )
    x1, y1 = start
    dx, dy = end[0] - x1, end[1] - y1
    lower, upper = 0.0, 1.0
    for p, q in (
        (-dx, x1 - left),
        (dx, right - x1),
        (-dy, y1 - top),
        (dy, bottom - y1),
    ):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        ratio = q / p
        if p < 0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return False
    return True


def proper_crossing(a, b) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    p1, p2 = a["start"], a["end"]
    q1, q2 = b["start"], b["end"]
    if any(
        math.dist(p, q) < 1e-6
        for p in (p1, p2)
        for q in (q1, q2)
    ):
        return False
    o1, o2 = orient(p1, p2, q1), orient(p1, p2, q2)
    o3, o4 = orient(q1, q2, p1), orient(q1, q2, p2)
    return o1 * o2 < -1e-10 and o3 * o4 < -1e-10


def ratsnest_edges(footprints):
    by_net = defaultdict(list)
    for footprint in footprints:
        reference = footprint.GetReference()
        for pad in footprint.Pads():
            net = pad.GetNetname()
            if net:
                point = pad.GetPosition()
                by_net[net].append(
                    (
                        point.x / 1e6,
                        point.y / 1e6,
                        reference,
                        pad.GetNumber(),
                    )
                )
    edges = []
    for net, points in sorted(by_net.items()):
        points.sort(key=lambda item: (item[2], item[3]))
        if len(points) < 2:
            continue
        remaining = set(range(1, len(points)))
        closest = [math.inf] * len(points)
        parent = [-1] * len(points)
        for index in remaining:
            closest[index] = math.dist(points[0][:2], points[index][:2])
            parent[index] = 0
        while remaining:
            index = min(remaining, key=lambda item: (closest[item], item))
            remaining.remove(index)
            first, second = points[parent[index]], points[index]
            length = math.dist(first[:2], second[:2])
            edges.append(
                {
                    "net": net,
                    "start": first[:2],
                    "end": second[:2],
                    "start_ref": first[2],
                    "end_ref": second[2],
                    "length": length,
                }
            )
            for other in remaining:
                distance = math.dist(points[index][:2], points[other][:2])
                if distance < closest[other]:
                    closest[other] = distance
                    parent[other] = index
    return edges


def score_board(footprints):
    edges = ratsnest_edges(footprints)
    wire_length = sum(edge["length"] for edge in edges)
    crossings = sum(
        edge_a["net"] != edge_b["net"] and proper_crossing(edge_a, edge_b)
        for index, edge_a in enumerate(edges)
        for edge_b in edges[index + 1 :]
    )
    obstacles = [
        (fp.GetReference(), footprint_box(fp))
        for fp in footprints
    ]
    blocked_length = 0.0
    for edge in edges:
        for reference, box in obstacles:
            if reference in (edge["start_ref"], edge["end_ref"]):
                continue
            if segment_intersects_box(
                edge["start"], edge["end"], box, OBSTACLE_MARGIN_MM
            ):
                blocked_length += edge["length"]
    score = (
        wire_length
        + CROSSING_WEIGHT_MM * crossings
        + BLOCKED_LENGTH_WEIGHT * blocked_length
    )
    return {
        "wire_length": wire_length,
        "crossings": int(crossings),
        "blocked_length": blocked_length,
        "score": score,
    }


def violation_signature(violation):
    items = tuple(
        sorted(
            (
                item.get("description", ""),
                round(item.get("pos", {}).get("x", 0.0), 4),
                round(item.get("pos", {}).get("y", 0.0), 4),
            )
            for item in violation.get("items", [])
        )
    )
    return (
        violation.get("type", ""),
        violation.get("severity", ""),
        violation.get("description", ""),
        items,
    )


def run_drc(cli: str, board_path: Path):
    with tempfile.TemporaryDirectory(prefix="rotation-drc-") as directory:
        report_path = Path(directory) / "drc.json"
        result = subprocess.run(
            [
                cli,
                "pcb",
                "drc",
                "--severity-all",
                "--format",
                "json",
                "--output",
                str(report_path),
                str(board_path),
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        if result.returncode or not report_path.is_file():
            detail = (result.stderr or result.stdout).strip()[-300:]
            raise RuntimeError(f"KiCad DRC failed: {detail}")
        return json.loads(report_path.read_text(encoding="utf-8"))


def run_drc_text(cli: str, board_path: Path, project_path: Path, text: str):
    with tempfile.TemporaryDirectory(prefix="rotation-trial-") as directory:
        trial_board = Path(directory) / board_path.name
        trial_project = Path(directory) / project_path.name
        trial_board.write_text(text, encoding="utf-8")
        shutil.copy2(project_path, trial_project)
        return run_drc(cli, trial_board)


def filter_safe_rotations(board, board_path, project_path, cli, source, result, baseline_drc):
    proposals = result["chosen_angles"]
    footprints = result["footprints"]
    by_reference = {fp.GetReference(): fp for fp in footprints}
    for footprint in result["movable"]:
        footprint.SetOrientationDegrees(result["original_angles"][footprint.GetReference()])

    baseline_signatures = {
        violation_signature(item) for item in baseline_drc.get("violations", [])
    }
    initial_metrics = score_board(footprints)
    individual_gains = {}
    for reference, angle in proposals.items():
        footprint = by_reference[reference]
        footprint.SetOrientationDegrees(angle)
        individual_gains[reference] = (
            initial_metrics["score"] - score_board(footprints)["score"]
        )
        footprint.SetOrientationDegrees(result["original_angles"][reference])

    ordered = sorted(proposals, key=lambda ref: (-individual_gains[ref], ref))
    accepted = {}
    accepted_steps = []
    rejected = {}
    trials = 0
    for _ in range(MAX_SAFE_PASSES):
        accepted_this_pass = False
        current_metrics = score_board(footprints)
        for reference in ordered:
            if reference in accepted or trials >= MAX_DRC_TRIALS:
                continue
            footprint = by_reference[reference]
            old_angle = round(footprint.GetOrientationDegrees()) % 360
            target_angle = proposals[reference]
            footprint.SetOrientationDegrees(target_angle)
            overlaps = courtyard_overlaps(footprints)
            outside = outside_board(footprints, board.GetBoardEdgesBoundingBox())
            candidate_metrics = score_board(footprints)
            if (
                overlaps - result["baseline_overlaps"]
                or outside - result["baseline_outside"]
                or candidate_metrics["score"] >= current_metrics["score"] - 1e-6
            ):
                footprint.SetOrientationDegrees(old_angle)
                continue

            trial_angles = dict(accepted)
            trial_angles[reference] = target_angle
            trial_text = patch_footprint_angles(source, trial_angles)
            if not balanced_sexpression(trial_text):
                raise ValueError(f"Trial rotation made the PCB unbalanced: {reference}")
            trial_drc = run_drc_text(cli, board_path, project_path, trial_text)
            trials += 1
            new_signatures = {
                violation_signature(item)
                for item in trial_drc.get("violations", [])
            } - baseline_signatures
            if new_signatures:
                rejected[reference] = len(new_signatures)
                footprint.SetOrientationDegrees(old_angle)
                continue

            accepted[reference] = target_angle
            rejected.pop(reference, None)
            accepted_steps.append(
                (reference, old_angle, target_angle, current_metrics, candidate_metrics)
            )
            current_metrics = candidate_metrics
            accepted_this_pass = True
        if not accepted_this_pass or trials >= MAX_DRC_TRIALS:
            break

    result["chosen_angles"] = accepted
    result["accepted_steps"] = accepted_steps
    result["rejected"] = rejected
    result["drc_trials"] = trials
    result["final_metrics"] = score_board(footprints)


def project_net_colors(project_path: Path, footprints) -> tuple[dict[str, str], int]:
    project = json.loads(project_path.read_text(encoding="utf-8"))
    colors = project.get("net_settings", {}).get("net_colors", {})
    if not isinstance(colors, dict):
        raise ValueError("net_settings.net_colors is not a mapping")
    nets = {
        pad.GetNetname()
        for footprint in footprints
        for pad in footprint.Pads()
        if pad.GetNetname()
    }
    missing = nets - colors.keys()
    if missing:
        raise ValueError(f"Configured net colors missing for: {sorted(missing)}")
    gnd_pads = sum(
        pad.GetNetname() == "GND"
        for footprint in footprints
        for pad in footprint.Pads()
    )
    return colors, gnd_pads


def refresh_svg(cli: str, board_path: Path, raw_path: Path, overlay_path: Path, colors):
    subprocess.run(
        [
            cli,
            "pcb",
            "export",
            "svg",
            "--mode-single",
            "--output",
            str(raw_path),
            "--layers",
            "F.Cu,In1.Cu,In2.Cu,B.Cu,Edge.Cuts",
            str(board_path),
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=True,
    )
    raw = raw_path.read_text(encoding="utf-8")
    ET.fromstring(raw)
    if not raw.rstrip().endswith("</svg>"):
        raise ValueError("Unexpected SVG root")

    board = pcbnew.LoadBoard(str(board_path))
    overlay = [
        '<g id="pad-net-overlay" aria-label="Pad-to-net color overlay" '
        'stroke="#111111" stroke-width="0.12" fill-opacity="0.86">'
    ]
    pad_count = 0
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            net = pad.GetNetname() or ""
            color = colors.get(net, "rgb(120, 120, 120)")
            x, y = pad.GetPosition().x / 1e6, pad.GetPosition().y / 1e6
            width, height = pad.GetSize().x / 1e6, pad.GetSize().y / 1e6
            angle, shape = pad.GetOrientationDegrees(), pad.GetShape()
            reference = html.escape(footprint.GetReference(), quote=True)
            number = html.escape(pad.GetNumber(), quote=True)
            escaped_net = html.escape(net, quote=True)
            overlay.append(
                f'<g data-ref="{reference}" data-pad="{number}" '
                f'data-net="{escaped_net}"><title>{reference}.{number} - '
                f'{escaped_net or "No net"}</title>'
            )
            if shape == 0:
                overlay.append(
                    f'<circle cx="{x:.4f}" cy="{y:.4f}" '
                    f'r="{min(width, height) / 2:.4f}" fill="{color}"/>'
                )
            elif shape in (2, 4):
                radius = min(width, height) / (2 if shape == 2 else 4)
                overlay.append(
                    f'<rect x="{x - width / 2:.4f}" y="{y - height / 2:.4f}" '
                    f'width="{width:.4f}" height="{height:.4f}" rx="{radius:.4f}" '
                    f'fill="{color}" transform="rotate({angle:.4f} {x:.4f} {y:.4f})"/>'
                )
            else:
                overlay.append(
                    f'<rect x="{x - width / 2:.4f}" y="{y - height / 2:.4f}" '
                    f'width="{width:.4f}" height="{height:.4f}" fill="{color}" '
                    f'transform="rotate({angle:.4f} {x:.4f} {y:.4f})"/>'
                )
            drill = pad.GetDrillSize()
            drill_x, drill_y = drill.x / 1e6, drill.y / 1e6
            if drill_x > 0 and drill_y > 0:
                overlay.append(
                    f'<ellipse cx="{x:.4f}" cy="{y:.4f}" '
                    f'rx="{drill_x / 2:.4f}" ry="{drill_y / 2:.4f}" '
                    f'fill="#ffffff" stroke="none" '
                    f'transform="rotate({angle:.4f} {x:.4f} {y:.4f})"/>'
                )
            overlay.append("</g>")
            pad_count += 1
    overlay.append("</g>")
    result = raw.rstrip()[:-6] + "\n" + "\n".join(overlay) + "\n</svg>\n"
    ET.fromstring(result)
    overlay_path.write_text(result, encoding="utf-8")
    return pad_count


def optimize(board, board_path: Path, project_path: Path, cli: str, raw_path: Path, overlay_path: Path):
    footprints = list(board.GetFootprints())
    by_reference = {fp.GetReference(): fp for fp in footprints}
    fixed_missing = FIXED_REFERENCES - by_reference.keys()
    if fixed_missing:
        raise ValueError(f"Expected fixed footprints are missing: {sorted(fixed_missing)}")

    colors, gnd_pad_count = project_net_colors(project_path, footprints)
    fixed = set(FIXED_REFERENCES)
    movable = [
        fp
        for fp in sorted(footprints, key=lambda item: item.GetReference())
        if fp.GetReference() not in fixed and not fp.IsLocked()
    ]
    original_angles = {
        fp.GetReference(): round(fp.GetOrientationDegrees()) % 360
        for fp in footprints
    }
    baseline_overlaps = courtyard_overlaps(footprints)
    baseline_outside = outside_board(footprints, board.GetBoardEdgesBoundingBox())
    initial = score_board(footprints)
    tested = 0
    accepted_steps = []

    for _ in range(MAX_PASSES):
        changed_this_pass = False
        for footprint in movable:
            reference = footprint.GetReference()
            current_angle = round(footprint.GetOrientationDegrees()) % 360
            best_angle = current_angle
            before = score_board(footprints)
            best_metrics = before
            for angle in ROTATIONS:
                tested += 1
                footprint.SetOrientationDegrees(angle)
                overlaps = courtyard_overlaps(footprints)
                outside = outside_board(footprints, board.GetBoardEdgesBoundingBox())
                if overlaps - baseline_overlaps or outside - baseline_outside:
                    continue
                metrics = score_board(footprints)
                if metrics["score"] < best_metrics["score"] - 1e-6:
                    best_angle, best_metrics = angle, metrics
            footprint.SetOrientationDegrees(best_angle)
            if best_angle != current_angle:
                accepted_steps.append(
                    (reference, current_angle, best_angle, before, best_metrics)
                )
                changed_this_pass = True
        if not changed_this_pass:
            break

    chosen_angles = {
        fp.GetReference(): round(fp.GetOrientationDegrees()) % 360
        for fp in movable
        if round(fp.GetOrientationDegrees()) % 360
        != original_angles[fp.GetReference()]
    }
    return {
        "footprints": footprints,
        "movable": movable,
        "colors": colors,
        "gnd_pads": gnd_pad_count,
        "original_angles": original_angles,
        "chosen_angles": chosen_angles,
        "baseline_overlaps": baseline_overlaps,
        "baseline_outside": baseline_outside,
        "initial_metrics": initial,
        "final_metrics": score_board(footprints),
        "tested": tested,
        "accepted_steps": accepted_steps,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parent.parent
    parser.add_argument("--pcb", type=Path, default=root / "stm32g4-canfd-sensor-node.kicad_pcb")
    parser.add_argument("--project", type=Path, default=root / "stm32g4-canfd-sensor-node.kicad_pro")
    parser.add_argument("--kicad-cli", default="")
    parser.add_argument("--raw-svg", type=Path, default=Path("/tmp/pcb_view_raw.svg"))
    parser.add_argument("--pad-svg", type=Path, default=Path("/tmp/pcb_view_pads.svg"))
    args = parser.parse_args()
    cli = find_cli(args.kicad_cli)
    check_safe_to_write(args.pcb)

    original_text = args.pcb.read_text(encoding="utf-8")
    if not balanced_sexpression(original_text):
        raise ValueError("Input PCB has unbalanced parentheses")
    original_hash = hashlib.sha256(original_text.encode("utf-8")).hexdigest()

    backup = create_backup(args.pcb)
    board = pcbnew.LoadBoard(str(args.pcb))
    baseline_drc = run_drc(cli, args.pcb)
    result = optimize(board, args.pcb, args.project, cli, args.raw_svg, args.pad_svg)
    proposed_count = len(result["chosen_angles"])
    filter_safe_rotations(
        board,
        args.pcb,
        args.project,
        cli,
        original_text,
        result,
        baseline_drc,
    )
    desired = result["chosen_angles"]

    if hashlib.sha256(args.pcb.read_bytes()).hexdigest() != original_hash:
        raise RuntimeError("PCB changed during optimization; refusing to overwrite it")
    if desired:
        check_safe_to_write(args.pcb)
        text = patch_footprint_angles(original_text, desired)
        if not balanced_sexpression(text):
            raise ValueError("Patched PCB has unbalanced parentheses")
        args.pcb.write_text(text, encoding="utf-8")
        if not balanced_sexpression(args.pcb.read_text(encoding="utf-8")):
            shutil.copy2(backup, args.pcb)
            raise RuntimeError("PCB balance check failed; restored the timestamped backup")
        loaded_board = pcbnew.LoadBoard(str(args.pcb))
        loaded_angles = {
            footprint.GetReference(): round(footprint.GetOrientationDegrees()) % 360
            for footprint in loaded_board.GetFootprints()
        }
        mismatches = {
            reference: (angle, loaded_angles.get(reference))
            for reference, angle in desired.items()
            if loaded_angles.get(reference) != angle
        }
        if mismatches:
            shutil.copy2(backup, args.pcb)
            raise RuntimeError(f"Saved footprint angles did not verify; restored backup: {mismatches}")

        final_drc = run_drc(cli, args.pcb)
        baseline_signatures = {
            violation_signature(item) for item in baseline_drc.get("violations", [])
        }
        new_violations = {
            violation_signature(item) for item in final_drc.get("violations", [])
        } - baseline_signatures
        if new_violations:
            shutil.copy2(backup, args.pcb)
            desired = {}
            result["chosen_angles"] = {}
            result["accepted_steps"] = []
            result["final_metrics"] = result["initial_metrics"]
            result["rolled_back"] = len(new_violations)
        else:
            result["rolled_back"] = 0
    else:
        final_drc = baseline_drc
        result["rolled_back"] = 0

    final_text = args.pcb.read_text(encoding="utf-8")
    if not balanced_sexpression(final_text):
        raise ValueError("Final PCB has unbalanced parentheses")

    final_board = pcbnew.LoadBoard(str(args.pcb))
    colors, gnd_pad_count = project_net_colors(args.project, final_board.GetFootprints())
    pad_count = refresh_svg(cli, args.pcb, args.raw_svg, args.pad_svg, colors)
    final_metrics = result["final_metrics"]
    initial_metrics = result["initial_metrics"]

    print(f"Timestamped backup: {backup}")
    print(
        f"Rotation tests: {result['tested']} across {len(result['movable'])} movable footprints; "
        f"proposed={proposed_count}, DRC trials={result.get('drc_trials', 0)}, "
        f"kept={len(result['chosen_angles'])}"
    )
    if result.get("rejected"):
        print(f"Rejected as DRC-unsafe: {len(result['rejected'])} candidates")
    if result["rolled_back"]:
        print(f"Rotations rolled back: {result['rolled_back']} new DRC violation signatures")
    for reference, old, new, before, after in result["accepted_steps"]:
        print(
            f"ROTATED {reference}: {old} -> {new}; "
            f"wire {before['wire_length']:.2f} -> {after['wire_length']:.2f} mm; "
            f"crossings {before['crossings']} -> {after['crossings']}"
        )
    if not result["chosen_angles"]:
        print("ROTATED: none; no candidate improved the objective and passed safety checks")
    print(
        f"Ratsnest estimate: wire={final_metrics['wire_length']:.2f} mm, "
        f"crossings={final_metrics['crossings']}, "
        f"blocked={final_metrics['blocked_length']:.2f} mm"
    )
    print(
        f"Pad net colors: {pad_count} overlay pads; "
        f"GND pads={gnd_pad_count} color={colors.get('GND', 'not configured')}; "
        "KiCad PCB format has no native per-pad color field"
    )
    print(f"balanced={balanced_sexpression(final_text)}")


if __name__ == "__main__":
    main()
