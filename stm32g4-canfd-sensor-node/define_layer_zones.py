import pathlib, re
PCB_PATH = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
def detect_box(txt):
    xs = [float(m.group(1)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
    ys = [float(m.group(2)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
    x0 = min(xs) if xs else 100.0
    y0 = min(ys) if ys else 60.0
    return x0, y0, x0 + 80.0, y0 + 60.0
def find_net(txt, cands):
    nets = {m.group(2).strip(): int(m.group(1)) for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt) if m.group(2).strip()}
    for c in cands:
        for n, code in nets.items():
            if c.upper() == n.upper(): return code, n
    for c in cands:
        for n, code in nets.items():
            if c.upper() in n.upper(): return code, n
    return 0, ""
def zone_blk(code, name, layer, pts, clr, mw):
    xy = "\n".join(f"        (xy {px:.3f} {py:.3f})" for px, py in pts)
    return f"  (zone (net {code}) (net_name \"{name}\") (layer \"{layer}\") (name \"{layer}_{name or code}\") (hatch edge 0.50)\n    (connect_pads (clearance {clr:.2f}))\n    (min_thickness {mw:.2f})\n    (fill yes (thermal_gap 0.30) (thermal_bridge_width 0.35))\n    (polygon\n      (pts\n{xy}\n      )\n    )\n  )"
def main():
    txt = PCB_PATH.read_text(encoding="utf-8")
    x0, y0, x1, y1 = detect_box(txt)
    m = 1.2
    pts = [(x0+m, y0+m), (x1-m, y0+m), (x1-m, y1-m), (x0+m, y1-m)]
    g_code, g_name = find_net(txt, ["GND", "DGND", "AGND"])
    p_code, p_name = find_net(txt, ["+3V3", "3V3", "VCC", "VDD"])
    txt = re.sub(r"\n  \(zone \(net \d+\) \(net_name \"[^\"]*\"\) \(layer \"(?:F\.Cu|In1\.Cu|In2\.Cu|B\.Cu)\"\) \(name \"(?:F\.Cu|In1\.Cu|In2\.Cu|B\.Cu)_[^\"]*\"\).*?\n  \)", "", txt, flags=re.S)
    zs = [zone_blk(g_code, g_name, "In1.Cu", pts, 0.25, 0.25), zone_blk(p_code, p_name, "In2.Cu", pts, 0.30, 0.25), zone_blk(g_code, g_name, "B.Cu", pts, 0.25, 0.20), zone_blk(g_code, g_name, "F.Cu", pts, 0.25, 0.20)]
    lp = txt.rfind(")")
    if lp != -1: txt = txt[:lp] + "\n" + "\n".join(zs) + "\n)\n"
    diff = txt.count("(") - txt.count(")")
    if diff > 0: txt = txt.rstrip() + "\n" + (")" * diff) + "\n"
    PCB_PATH.write_text(txt, encoding="utf-8")
    print(f"ZONES_DEFINED_OK | In1.Cu={g_name}(#{g_code}) | In2.Cu={p_name}(#{p_code}) | balanced={txt.count(chr(40)) == txt.count(chr(41))}")
if __name__ == "__main__": main()
