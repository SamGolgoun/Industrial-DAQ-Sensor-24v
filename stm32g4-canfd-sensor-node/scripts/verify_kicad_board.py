#!/usr/bin/env python3
"""Read-only, concise KiCad PCB checks. Requires Python 3 and kicad-cli."""

import argparse
import json
import math
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

TOKEN = re.compile(r'\s+|;[^\n]*|"(?:\\.|[^"\\])*"|[()]|[^\s()]+')
COPPER = {"F.Cu", "In1.Cu", "In2.Cu", "B.Cu"}
COPPER_ERRORS = {
    "clearance", "shorting_items", "track_width", "copper_edge_clearance",
    "via_dangling", "zones_intersect",
}


def parse_sexp(text):
    tokens = [m.group() for m in TOKEN.finditer(text)
              if not m.group().isspace() and not m.group().startswith(";")]
    i = 0

    def item():
        nonlocal i
        token = tokens[i]
        i += 1
        if token != "(":
            return json.loads(token) if token.startswith('"') else token
        result = []
        while tokens[i] != ")":
            result.append(item())
        i += 1
        return result

    return item()


def nodes(value, name=None):
    if isinstance(value, list):
        if value and isinstance(value[0], str) and (name is None or value[0] == name):
            yield value
        for child in value:
            yield from nodes(child, name)


def direct(node, name):
    return next((x for x in node[1:] if isinstance(x, list) and x and x[0] == name), None)


def number(node, key, default=None):
    entry = direct(node, key)
    try:
        return float(entry[1]) if entry else default
    except (TypeError, ValueError):
        return default


def project_rules(value):
    if isinstance(value, dict):
        if "min_clearance" in value and "min_track_width" in value:
            return value
        for child in value.values():
            found = project_rules(child)
            if found:
                return found
    elif isinstance(value, list):
        for child in value:
            found = project_rules(child)
            if found:
                return found
    return {}


def pad_hits_via(via, footprint):
    fp_at = direct(footprint, "at")
    if not fp_at:
        return False
    fx, fy = map(float, fp_at[1:3])
    fp_angle = float(fp_at[3]) if len(fp_at) > 3 else 0.0
    via_at = direct(via, "at")
    vx, vy = map(float, via_at[1:3])
    va = math.radians(-fp_angle)
    dx, dy = vx - fx, vy - fy
    local_via = (dx * math.cos(va) + dy * math.sin(va),
                 -dx * math.sin(va) + dy * math.cos(va))
    via_layers = set(direct(via, "layers")[1:])

    for pad in (x for x in footprint[1:] if isinstance(x, list) and x and x[0] == "pad"):
        pad_at, size, layers = direct(pad, "at"), direct(pad, "size"), direct(pad, "layers")
        if not pad_at or not size or not layers:
            continue
        pad_layers = set(layers[1:])
        if "*.Cu" not in pad_layers and not (pad_layers & via_layers):
            continue
        px, py = map(float, pad_at[1:3])
        pa = float(pad_at[3]) if len(pad_at) > 3 else 0.0
        ox, oy = px * math.cos(math.radians(fp_angle)) + py * math.sin(math.radians(fp_angle)), \
                 -px * math.sin(math.radians(fp_angle)) + py * math.cos(math.radians(fp_angle))
        rx, ry = local_via[0] - ox, local_via[1] - oy
        a = math.radians(-pa)
        x, y = rx * math.cos(a) + ry * math.sin(a), -rx * math.sin(a) + ry * math.cos(a)
        w, h = map(float, size[1:3])
        shape = pad[3] if len(pad) > 3 else "rect"
        if shape in ("circle",):
            hit = x * x + y * y <= (min(w, h) / 2) ** 2
        elif shape == "oval":
            if w >= h:
                hit = max(abs(x) - (w - h) / 2, 0) ** 2 + y * y <= (h / 2) ** 2
            else:
                hit = x * x + max(abs(y) - (h - w) / 2, 0) ** 2 <= (w / 2) ** 2
        else:
            hit = abs(x) <= w / 2 and abs(y) <= h / 2
        if hit:
            return True
    return False


def main():
    here = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pcb", type=Path, default=next(here.glob("*.kicad_pcb"), None))
    ap.add_argument("--project", type=Path, default=next(here.glob("*.kicad_pro"), None))
    default_cli = shutil.which("kicad-cli") or ""
    if not default_cli:
        mounts = sorted(Path("/tmp").glob(".mount_kicad*/usr/bin/kicad-cli"))
        if mounts:
            default_cli = str(mounts[-1])
    ap.add_argument("--kicad-cli", default=default_cli)
    ap.add_argument("--outline", default="80x60", help="expected Edge.Cuts size in mm")
    args = ap.parse_args()
    results = []

    def check(name, passed, detail=""):
        results.append((name, bool(passed), detail))

    if not args.pcb or not args.project or not args.pcb.is_file() or not args.project.is_file():
        check("inputs", False, "PCB/project file missing")
        args.pcb = args.pcb or Path("missing.kicad_pcb")
        args.project = args.project or Path("missing.kicad_pro")
    else:
        pcb_text = args.pcb.read_text(encoding="utf-8")
        board = parse_sexp(pcb_text)
        project = json.loads(args.project.read_text(encoding="utf-8"))
        rules = project_rules(project)
        layer_node = direct(board, "layers")
        layers = {row[1] for row in layer_node[1:] if isinstance(row, list) and len(row) > 1} if layer_node else set()
        segments = list(nodes(board, "segment"))
        vias = list(nodes(board, "via"))
        used = {direct(s, "layer")[1] for s in segments if direct(s, "layer")}
        check("4-layer stack and routing", COPPER <= layers and COPPER <= used and bool(vias),
              f"layers={len(COPPER & layers)}/4, routed={len(COPPER & used)}/4, vias={len(vias)}")

        edge = [x for x in board[1:] if isinstance(x, list) and x and x[0].startswith("gr_")
                and direct(x, "layer") and direct(x, "layer")[1] == "Edge.Cuts"]
        points = []
        for obj in edge:
            for p in nodes(obj):
                if p and p[0] in ("start", "end", "mid", "xy") and len(p) >= 3:
                    try:
                        points.append((float(p[1]), float(p[2])))
                    except (TypeError, ValueError):
                        pass
        dims_ok = False
        if points:
            width = max(x for x, _ in points) - min(x for x, _ in points)
            height = max(y for _, y in points) - min(y for _, y in points)
            try:
                ew, eh = map(float, args.outline.lower().split("x"))
                dims_ok = abs(width - ew) < 0.1 and abs(height - eh) < 0.1
            except ValueError:
                pass
            outline_detail = f"{width:.2f}x{height:.2f} mm"
        else:
            outline_detail = "no Edge.Cuts geometry"
        rules_ok = (rules.get("min_clearance") == 0.15 and rules.get("min_track_width") == 0.15)
        check("outline and 0.15 mm rules", dims_ok and rules_ok,
              f"{outline_detail}; clearance={rules.get('min_clearance')}, width={rules.get('min_track_width')}")

        widths = [number(s, "width") for s in segments]
        widths = [w for w in widths if w is not None]
        min_width = min(widths, default=0.0)
        check("track widths", rules_ok and min_width >= 0.15 - 1e-6
              and any(abs(w - 0.15) <= 0.01 for w in widths)
              and any(abs(w - 0.30) <= 0.02 for w in widths),
              f"min={min_width:.3f}; 0.15={sum(abs(w-.15)<=.01 for w in widths)}; "
              f"0.30={sum(abs(w-.30)<=.02 for w in widths)}; 0.40-0.60={sum(.4<=w<=.6 for w in widths)}")
        power_nets = {"/24V_IN", "24V_IN", "/+24V_PROT", "+24V_PROT", "+3V3", "+3V3_ANA"}
        power_widths = [number(s, "width") for s in segments
                        if direct(s, "net") and len(direct(s, "net")) > 1
                        and direct(s, "net")[1] in power_nets]
        check("power track sizing", any(w is not None and 0.40 <= w <= 0.60 for w in power_widths),
              f"power tracks={len(power_widths)}, 0.40-0.60 mm="
              f"{sum(w is not None and .4<=w<=.6 for w in power_widths)}")

        hits = 0
        footprints = list(nodes(board, "footprint"))
        for via in vias:
            if any(pad_hits_via(via, fp) for fp in footprints):
                hits += 1
        check("no via-in-pad", hits == 0, f"overlapping vias={hits}")

        zones = list(nodes(board, "zone"))
        gnd_zones = []
        for z in zones:
            net = direct(z, "net_name") or direct(z, "net")
            if net and len(net) > 1 and net[1].strip('"') == "GND":
                gnd_zones.append(z)
        filled = sum(bool(direct(z, "fill")) for z in gnd_zones)
        check("filled GND zone", bool(gnd_zones) and filled == len(gnd_zones),
              f"zones={len(gnd_zones)}, filled={filled}")

    cli = args.kicad_cli or ""
    if not cli:
        check("KiCad DRC", False, "set --kicad-cli or add kicad-cli to PATH")
    else:
        try:
            with tempfile.TemporaryDirectory(prefix="kicad-verify-") as tmp:
                report = Path(tmp) / "drc.json"
                run = subprocess.run(
                    [cli, "pcb", "drc", "--refill-zones", "--save-board", "--severity-all", "--format", "json",
                     "--output", str(report), str(args.pcb)],
                    capture_output=True, text=True, timeout=60, check=False,
                )
                if run.returncode or not report.is_file():
                    check("KiCad DRC", False, (run.stderr or run.stdout).strip()[-180:])
                else:
                    drc = json.loads(report.read_text())
                    violations = drc.get("violations", [])
                    unconnected = drc.get("unconnected_items", [])
                    copper = [v for v in violations if v.get("type") in COPPER_ERRORS]
                    gnd_open = sum("GND" in " ".join(i.get("description", "") for i in v.get("items", []))
                                   for v in unconnected)
                    check("zero unconnected", not unconnected, str(len(unconnected)))
                    check("GND continuity", gnd_open == 0, f"open GND connections={gnd_open}")
                    check("zero copper DRC errors", not copper, f"errors={len(copper)}")
        except subprocess.TimeoutExpired:
            check("KiCad DRC", False, "timed out after 60s")
        except OSError as exc:
            check("KiCad DRC", False, str(exc))

    for name, passed, detail in results:
        print(f"{'PASS' if passed else 'FAIL'} {name}" + (f": {detail}" if detail else ""))
    failed = sum(not passed for _, passed, _ in results)
    print(f"RESULT: {'PASS' if not failed else 'FAIL'} ({len(results)-failed}/{len(results)} tests)")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
