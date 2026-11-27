import glob
import importlib.util
import json
import re
import subprocess
import sys
from collections import Counter
from pathlib import Path

board = Path(sys.argv[1])
if list(board.parent.glob("~*.kicad_pcb.lck")):
    print("LOCKED")
    raise SystemExit(2)
cli = sorted(glob.glob("/tmp/.mount_kicad*/usr/bin/kicad-cli"))[-1]
router = Path(__file__).with_name("route_board.py")
report = Path("/tmp/safe-route-drc.json")
spec = importlib.util.spec_from_file_location(
    "board_verifier", Path.cwd() / "scripts/verify_kicad_board.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def drc():
    if list(board.parent.glob("~*.kicad_pcb.lck")):
        raise RuntimeError("KiCad PCB lock appeared")
    result = subprocess.run(
        [cli, "pcb", "drc", "--refill-zones", "--save-board", "--severity-all",
         "--format", "json", "--output", str(report), str(board)],
        capture_output=True, text=True, timeout=60,
    )
    if result.returncode not in (0, 5) or not report.is_file():
        raise RuntimeError("KiCad DRC failed")
    return json.loads(report.read_text())


def sigs(data):
    return Counter((v.get("type"), v.get("description"))
                   for v in data.get("violations", []))


def via_pad_overlaps():
    parsed = module.parse_sexp(board.read_text())
    footprints = list(module.nodes(parsed, "footprint"))
    return sum(
        any(module.pad_hits_via(via, fp) for fp in footprints)
        for via in module.nodes(parsed, "via")
    )


def nets(data):
    names = set()
    for item in data.get("unconnected_items", []):
        for entry in item.get("items", []):
            names.update(re.findall(r"\[([^\]]+)\]", entry.get("description", "")))
    return sorted(n for n in names if not n.startswith("unconnected-"))


current = drc()
accepted = 0
attempted = 0
for _ in range(3):
    before_round = len(current.get("unconnected_items", []))
    for net in nets(current):
        if list(board.parent.glob("~*.kicad_pcb.lck")):
            print("LOCKED")
            raise SystemExit(2)
        original = board.read_bytes()
        attempted += 1
        route = subprocess.run(
            [sys.executable, str(router), str(board), net],
            capture_output=True, text=True, timeout=25,
        )
        if route.returncode or "routed 0" in route.stdout:
            board.write_bytes(original)
            continue
        trial = drc()
        if sigs(trial) - sigs(current) or via_pad_overlaps():
            board.write_bytes(original)
            continue
        if len(trial.get("unconnected_items", [])) >= len(current.get("unconnected_items", [])):
            board.write_bytes(original)
            continue
        current = trial
        accepted += 1
    if len(current.get("unconnected_items", [])) >= before_round:
        break

print(f"accepted_routes={accepted}/{attempted}")
print(f"remaining_unconnected={len(current.get('unconnected_items', []))}")
