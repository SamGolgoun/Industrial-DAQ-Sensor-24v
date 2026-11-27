import glob
import importlib.util
import json
import math
import re
import subprocess
import sys
import uuid
from collections import Counter
from pathlib import Path

board = Path(sys.argv[1])
if list(board.parent.glob("~*.kicad_pcb.lck")):
    print("LOCKED")
    raise SystemExit(2)
cli = sorted(glob.glob("/tmp/.mount_kicad*/usr/bin/kicad-cli"))[-1]
report = Path("/tmp/gnd-stitch-drc.json")
verifier = Path.cwd() / "scripts/verify_kicad_board.py"
spec = importlib.util.spec_from_file_location("board_verifier", verifier)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def run_drc():
    result = subprocess.run(
        [cli, "pcb", "drc", "--refill-zones", "--save-board", "--severity-all",
         "--format", "json", "--output", str(report), str(board)],
        capture_output=True, text=True, timeout=60,
    )
    if result.returncode not in (0, 5) or not report.is_file():
        raise RuntimeError("KiCad DRC failed")
    return json.loads(report.read_text())


def signatures(drc):
    return Counter((v.get("type"), v.get("description")) for v in drc.get("violations", []))


def via_pad_overlaps():
    parsed = module.parse_sexp(board.read_text())
    footprints = list(module.nodes(parsed, "footprint"))
    return sum(
        any(module.pad_hits_via(via, fp) for fp in footprints)
        for via in module.nodes(parsed, "via")
    )


baseline = run_drc()
baseline_overlaps = via_pad_overlaps()
gnd_pads = set()
for item in baseline.get("unconnected_items", []):
    for subitem in item.get("items", []):
        desc = subitem.get("description", "")
        match = re.search(r"Pad (\S+) \[GND\] of (\S+) on", desc)
        if match:
            pos = subitem.get("pos", {})
            gnd_pads.add((match.group(2), match.group(1), float(pos["x"]), float(pos["y"])))

accepted = 0
for ref, pad, x, y in sorted(gnd_pads):
    if list(board.parent.glob("~*.kicad_pcb.lck")):
        print("LOCKED")
        raise SystemExit(2)
    original = board.read_bytes()
    added = False
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                   (0.7071, 0.7071), (-0.7071, 0.7071),
                   (0.7071, -0.7071), (-0.7071, -0.7071)):
        vx, vy = round(x + dx * 2.5, 4), round(y + dy * 2.5, 4)
        seg_uuid, via_uuid = str(uuid.uuid4()), str(uuid.uuid4())
        nodes = (
            f'\n(segment (start {x:.4f} {y:.4f}) (end {vx:.4f} {vy:.4f}) '
            f'(width 0.15) (layer "F.Cu") (net "GND") (uuid "{seg_uuid}"))\n'
            f'(via (at {vx:.4f} {vy:.4f}) (size 0.6) (drill 0.3) '
            f'(layers "F.Cu" "B.Cu") (net "GND") (uuid "{via_uuid}"))\n'
        ).encode()
        data = original.rstrip()
        if not data.endswith(b")"):
            raise RuntimeError("Malformed board ending")
        board.write_bytes(data[:-1] + nodes + b")\n")
        try:
            trial = run_drc()
        except Exception:
            board.write_bytes(original)
            raise
        if signatures(trial) - signatures(baseline) or via_pad_overlaps() > baseline_overlaps:
            board.write_bytes(original)
            continue
        baseline = trial
        accepted += 1
        added = True
        break
    if not added:
        board.write_bytes(original)

print(f"accepted_gnd_pad_routes={accepted}/{len(gnd_pads)}")
print(f"remaining_unconnected={len(baseline.get('unconnected_items', []))}")
