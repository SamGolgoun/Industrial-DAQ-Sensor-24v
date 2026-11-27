import json
import pathlib
import re
import subprocess

pcb_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
drc_path = pathlib.Path("/tmp/drc_j2_clean.json")
text = pcb_path.read_text(encoding="utf-8")

# Find right-most Edge.Cuts coordinate
edge_xs = [
    float(x)
    for x in re.findall(r"\(\s*(?:start|end|mid)\s+(-?\d+(?:\.\d+)?)\s+-?\d+(?:\.\d+)?\s*\)", text)
]
x1 = max(edge_xs) if edge_xs else 180.0

# Locate J2 reference inside a footprint block and pull its X position inward
pattern = re.compile(
    r'(\(footprint\b[\s\S]{0,800}?property\s+"Reference"\s+"J2"[\s\S]{0,600}?\(\s*at\s+)(-?\d+(?:\.\d+)?)(\s+-?\d+(?:\.\d+)?)',
    re.MULTILINE,
)

def repl(match):
    prefix = match.group(1)
    old_x = float(match.group(2))
    y_part = match.group(3)
    new_x = round(min(old_x - 4.0, x1 - 6.0), 3)
    print(f"Moved J2: X {old_x} -> {new_x}, Y{y_part} (right edge x1={x1})")
    return f"{prefix}{new_x}{y_part}"

new_text, count = pattern.subn(repl, text, count=1)
if count == 0:
    print("Warning: J2 footprint placement block not matched.")
else:
    pcb_path.write_text(new_text, encoding="utf-8")

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
print(f"\n=== DRC VIOLATIONS ({len(viols)}) ===")
for idx, v in enumerate(viols, 1):
    print(f"  {idx}. [{v.get('severity', 'error')}] ({v.get('type', '?')}) {v.get('description', '')}")
