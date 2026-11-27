import json, pathlib, re
pcb_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
pro_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pro")
txt = pcb_path.read_text(encoding="utf-8")
nets = sorted({m.group(2) for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt) if m.group(2)})
def pick_color(name):
    u = name.upper()
    if "GND" in u: return "rgb(46, 204, 113)"
    if "CAN" in u or "FDCAN" in u: return "rgb(243, 156, 18)"
    if "24V" in u or "VIN" in u or "VBUS" in u: return "rgb(231, 76, 60)"
    if "3V3" in u or "VDD" in u or "VCC" in u: return "rgb(230, 126, 34)"
    if "ADC" in u or "ANA" in u or "OSC" in u or "CLK" in u: return "rgb(155, 89, 182)"
    return "rgb(52, 152, 219)"
net_colors = {n: pick_color(n) for n in nets}
pro = json.loads(pro_path.read_text(encoding="utf-8")) if pro_path.exists() else {}
pcb_sec = pro.setdefault("pcb", {})
pcb_sec["net_colors"] = net_colors
if pro_path.exists(): pro_path.write_text(json.dumps(pro, indent=2) + "\n", encoding="utf-8")
diff = txt.count("(") - txt.count(")")
if diff > 0: txt = txt.rstrip() + "\n" + (")" * diff) + "\n"; pcb_path.write_text(txt, encoding="utf-8")
summary = {}
for n, c in net_colors.items(): summary[c] = summary.get(c, 0) + 1
print("Total colored nets:", len(net_colors), "| Balanced PCB parens:", txt.count("(") == txt.count(")"))
for c, cnt in summary.items(): print(" ", c, "->", cnt, "nets")
