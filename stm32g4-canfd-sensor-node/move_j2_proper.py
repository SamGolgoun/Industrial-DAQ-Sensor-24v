import json
import pathlib
import re
import subprocess

pcb_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
drc_path = pathlib.Path("/tmp/drc_j2_proper.json")
text = pcb_path.read_text(encoding="utf-8")

# Find right-most Edge.Cuts coordinate
edge_xs = [
    float(x)
    for x in re.findall(r"\(\s*(?:start|end|mid)\s+(-?\d+(?:\.\d+)?)\s+-?\d+(?:\.\d+)?\s*\)", text)
]
x1 = max(edge_xs) if edge_xs else 180.0

# Split by top-level "(footprint " blocks so we never touch inner pad "(at ...)" entries
parts = text.split("\n  (footprint ")
updated_count = 0

for idx in range(1, len(parts)):
    chunk = parts[idx]
    if '"Reference" "J2"' in chunk or '(fp_text reference "J2"' in chunk or "(fp_text reference J2" in chunk:
        m_at = re.search(r"(\(\s*at\s+)(-?\d+(?:\.\d+)?)(\s+-?\d+(?:\.\d+)?)(\s+-?\d+(?:\.\d+)?)?(\s*\))", chunk)
        if m_at:
            old_x = float(m_at.group(2))
            old_y = float(m_at.group(3))
            ang_part = m_at.group(4) or ""
            close_part = m_at.group(5)
            new_x = round(min(old_x - 5.0, x1 - 7.0), 3)
            new_at = f"{m_at.group(1)}{new_x}{m_at.group(3)}{ang_part}{close_part}"
            parts[idx] = chunk[:m_at.start()] + new_at + chunk[m_at.end():]
            updated_count += 1
            print(f"J2 moved: ({old_x}, {old_y}) -> ({new_x}, {old_y}) | board right edge x1={x1}")
            break

if updated_count == 0:
    print("J2 footprint was not matched in file.")
else:
    pcb_path.write_text("\n  (footprint ".join(parts), encoding="utf-8")

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
for i, v in enumerate(viols, 1):
    print(f"  {i}. [{v.get('severity', 'error')}] ({v.get('type', '?')}) {v.get('description', '')}")
