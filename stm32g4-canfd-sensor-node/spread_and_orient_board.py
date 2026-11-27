import math, pathlib, re
PCB_PATH = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
def find_end(s, idx):
    d = q = esc = 0
    for i in range(idx, len(s)):
        c = s[i]
        if esc: esc = 0
        elif c == chr(92): esc = 1
        elif c == chr(34): q ^= 1
        elif not q:
            d += (c == "(") - (c == ")")
            if d == 0: return i
    return -1
def iter_fps(s):
    er = find_end(s, 0); i = 1
    while i < er:
        if s[i] == "(":
            e = find_end(s, i)
            if e < 0: break
            b = s[i:e+1]
            if b.startswith("(footprint "): yield i, e+1, b
            i = e + 1
        else: i += 1
def parse_fp(st, en, b):
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1).strip() if rm else ""
    fx = fy = fdeg = 0.0; at_sp = None
    i = 1; d = 1; q = False; esc = False
    while i < len(b):
        c = b[i]
        if esc: esc = 0
        elif c == chr(92): esc = 1
        elif c == chr(34): q ^= 1
        elif not q:
            if c == "(":
                if d == 1 and re.match(r"^\(\s*at\b", b[i:i+10]):
                    ae = find_end(b, i)
                    am = re.match(r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$", b[i:ae+1])
                    if am:
                        fx, fy = float(am.group(1)), float(am.group(2))
                        fdeg = float(am.group(3)) if am.group(3) else 0.0
                        at_sp = (i, ae+1); break
                d += 1
            elif c == ")": d -= 1
        i += 1
    nets = [m.group(1).strip() for m in re.finditer(r"\(\s*net\s+\d+\s+\"([^\"]*)\"\s*\)", b) if m.group(1).strip() and "GND" not in m.group(1).upper()]
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp, "nets": nets}
def detect_box(txt):
    xs = [float(m.group(1)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
    ys = [float(m.group(2)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
    x0 = min(xs) if xs else 100.0; y0 = min(ys) if ys else 60.0
    return x0, y0, x0 + 80.0, y0 + 60.0
def best_angle(fp, all_fps):
    if not fp["nets"]: return fp["deg"]
    net_set = set(fp["nets"])
    peers = [o for o in all_fps if o["ref"] != fp["ref"] and (set(o["nets"]) & net_set)]
    if not peers: return fp["deg"]
    cx = sum(p["x"] for p in peers) / len(peers)
    cy = sum(p["y"] for p in peers) / len(peers)
    ang = math.degrees(math.atan2(cy - fp["y"], cx - fp["x"]))
    snapped = round(ang / 90.0) * 90.0
    return float(snapped % 360.0)
def main():
    txt = PCB_PATH.read_text(encoding="utf-8")
    x0, y0, x1, y1 = detect_box(txt)
    fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
    by_ref = {f["ref"]: f for f in fps if f["ref"]}
    orig = {r: (f["x"], f["y"], f["deg"]) for r, f in by_ref.items()}
    fixed = {
        "H1": (x0 + 5.0, y0 + 5.0), "H2": (x1 - 5.0, y0 + 5.0),
        "H3": (x1 - 5.0, y1 - 5.0), "H4": (x0 + 5.0, y1 - 5.0),
        "FID1": (x0 + 13.5, y0 + 7.5), "FID2": (x1 - 13.5, y0 + 7.5), "FID3": (x1 - 13.5, y1 - 7.5),
        "J2": (x1 - 12.5, y0 + 14.5), "J4": (x0 + 44.0, y1 - 10.5),
        "U1": (x0 + 49.0, y0 + 27.5), "Y1": (x0 + 44.2, y0 + 24.0),
    }
    for r, xy in fixed.items():
        if r in by_ref: by_ref[r]["x"], by_ref[r]["y"] = xy
    resistors, caps, noisy_pwr, noisy_io, sens = [], [], [], [], []
    for f in fps:
        r = f["ref"]
        if not r or r in fixed: continue
        if r.startswith("R"): resistors.append(f)
        elif r.startswith(("L", "FB", "Q", "D", "F", "SW")): noisy_pwr.append(f)
        elif r in {"U2", "U3", "U4", "U5"} or r.startswith("J"): noisy_io.append(f)
        elif re.search(r"ADC|ANA|VREF|DAC", f["b"], re.I) or r.startswith("U"): sens.append(f)
        elif r.startswith("C"): caps.append(f)
        else: noisy_io.append(f)
    def spread(items, gx0, gy0, cols, dx, dy):
        for idx, it in enumerate(items):
            it["x"] = round(gx0 + (idx % cols) * dx, 3)
            it["y"] = round(gy0 + (idx // cols) * dy, 3)
    spread(noisy_pwr, x0 + 11.5, y0 + 14.5, cols=4, dx=5.4, dy=5.4)
    spread(resistors, x0 + 11.5, y1 - 24.5, cols=6, dx=6.0, dy=5.4)
    spread(caps[:8], x0 + 39.5, y0 + 35.0, cols=4, dx=4.6, dy=4.4)
    spread(caps[8:], x0 + 12.0, y0 + 27.5, cols=5, dx=5.2, dy=5.0)
    spread(sens, x0 + 40.0, y0 + 16.5, cols=3, dx=5.2, dy=4.8)
    spread(noisy_io, x1 - 25.0, y0 + 21.0, cols=2, dx=6.4, dy=6.2)
    rotatable = [f for f in fps if f["ref"] and f["ref"] not in {"H1","H2","H3","H4","FID1","FID2","FID3","J2","J4","U1","Y1"}]
    for f in rotatable:
        f["deg"] = best_angle(f, fps)
    changed = []
    for r, f in sorted(by_ref.items()):
        ox, oy, odeg = orig[r]
        if abs(f["x"] - ox) > 0.01 or abs(f["y"] - oy) > 0.01 or abs(f["deg"] - odeg) > 0.01:
            changed.append((r, round(ox,2), round(oy,2), round(f["x"],2), round(f["y"],2), int(odeg), int(f["deg"])))
    edits = []
    for f in fps:
        if f["at_sp"]:
            sa, ea = f["at_sp"]
            d_str = int(f["deg"]) if float(f["deg"]).is_integer() else f["deg"]
            na = f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f} {d_str})" if f["deg"] != 0 else f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f})"
            edits.append((f["st"], f["en"], f["b"][:sa] + na + f["b"][ea:]))
    for s, e, nb in sorted(edits, key=lambda item: item[0], reverse=True):
        txt = txt[:s] + nb + txt[e:]
    diff = txt.count("(") - txt.count(")")
    if diff > 0: txt = txt.rstrip() + "\n" + (")" * diff) + "\n"
    PCB_PATH.write_text(txt, encoding="utf-8")
    print(f"SPREAD_ORIENT_OK | changed={len(changed)} | balanced={txt.count(chr(40)) == txt.count(chr(41))}")
    for item in changed[:25]:
        print(f"  {item[0]}: ({item[1]},{item[2]}) -> ({item[3]},{item[4]}) rot {item[5]}->{item[6]}")
if __name__ == "__main__": main()
