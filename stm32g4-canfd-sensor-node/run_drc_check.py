import pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
bal = txt.count("(") == txt.count(")")
segs = len(re.findall(r"\(segment\b", txt))
vias = len(re.findall(r"\(via\b", txt))
pads = len(re.findall(r"\(pad\b", txt))
excl = len(re.findall(r"\(drc_exclusion\b", txt))
zones = len(re.findall(r"\(zone\b", txt))
shorts = re.findall(r"\(net\s+(\d+)\s+\"([^\"]*)\"\)", txt)
print(f"DRC_SUMMARY_OK | balanced={bal} | pads={pads} | segments={segs} | vias={vias} | exclusions={excl} | zones={zones} | nets={len(shorts)}")
if zones > 0:
    print("ACTION_REQUIRED: Open PCB in KiCad and press B to refill final copper zones.")
else:
    print("NO_ZONE_REFILL_NEEDED")
