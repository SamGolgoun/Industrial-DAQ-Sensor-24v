import pathlib, re

p = pathlib.Path("stm32g4-canfd-sensor-node_v2.kicad_pcb")
text = p.read_text()

targets = {
"U2": "(at 132 90)",
"Y1": "(at 121.5 84.5)",
"C18": "(at 121 81.5)",
"C15": "(at 121 87.5)",
"SW1": "(at 143 77)",
"C17": "(at 146.5 77)",
}

for ref, new_at in targets.items():
fp_pat = re.compile(
    rf'(\(footprint\b[^\n]*\n\s*)\(at\s+[^\)]+\)(.*?\(property\s+"Reference"\s+"{re.escape(ref)}"\b)',
    re.DOTALL,
)
text, n = fp_pat.subn(rf'\g<1>{new_at}\g<2>', text, count=1)
if n:
    print(f"Updated {ref} -> {new_at}")

p.write_text(text)
