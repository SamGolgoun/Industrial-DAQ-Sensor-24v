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
    nets = {m.group(1).strip() for m in re.finditer(r"\(\s*net\s+\d+\s+\"([^\"]*)\"\s*\)", b) if m.group(1).strip() and "GND" not in m.group(1).upper()}
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp, "nets": nets}
def detect_box(txt):
    xs = [float(m.group(1)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
    ys = [float(m.group(2)) for m in re.finditer(r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts", txt)]
    x0 = min(xs) if xs else 100.0; y0 = min(ys) if ys else 60.0
    return x0, y0, x0 + 80.0, y0 + 60.0
def main():
    txt = PCB_PATH.read_text(encoding="utf-8")
    x0, y0, x1, y1 = detect_box(txt)
    fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
    by_ref = {f["ref"]: f for f in fps if f["ref"]}
    protected = {"H1","H2","H3","H4","FID1","FID2","FID3","J2","J4","U1","Y1"}
    targets = [f for f in fps if f["ref"] and not f["ref"].startswith("C") and not f["ref"].startswith(("H","FID"))]
    caps = [f for f in fps if f["ref"].startswith("C")]
    placed_coords = [(f["x"], f["y"]) for f in fps if f["ref"] and not f["ref"].startswith("C")]
    moved_caps = []
    for idx, cap in enumerate(caps):
        shared = [t for t in targets if (t["nets"] & cap["nets"])]
        host = shared[0] if shared else (by_ref.get("U1") or (targets[0] if targets else None))
        if not host:
            continue
        ox, oy = cap["x"], cap["y"]
        ang = (idx * 0.85) % (2.0 * math.pi)
        rad = 3.8 + (idx % 3) * 0.45
        for step in range(12):
            a = ang + step * 0.55
            nx = max(x0 + 8.0, min(x1 - 8.0, host["x"] + rad * math.cos(a)))
            ny = max(y0 + 8.0, min(y1 - 8.0, host["y"] + rad * math.sin(a)))
            if all(math.hypot(nx - px, ny - py) >= 3.2 for px, py in placed_coords):
                cap["x"], cap["y"] = round(nx, 3), round(ny, 3)
                placed_coords.append((cap["x"], cap["y"]))
                break
        else:
            cap["x"], cap["y"] = round(max(x0 + 8.0, min(x1 - 8.0, ox)), 3), round(max(y0 + 8.0, min(y1 - 8.0, oy)), 3)
            placed_coords.append((cap["x"], cap["y"]))
        if abs(cap["x"] - ox) > 0.01 or abs(cap["y"] - oy) > 0.01:
            moved_caps.append((cap["ref"], host["ref"], round(ox,2), round(oy,2), round(cap["x"],2), round(cap["y"],2)))
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
    print(f"DECOUP_CAPS_OK | moved_caps={len(moved_caps)}/{len(caps)} | balanced={txt.count(chr(40)) == txt.count(chr(41))}")
    for m in moved_caps[:25]:
        print(f"  {m[0]} -> near {m[1]}: ({m[2]},{m[3]}) to ({m[4]},{m[5]})")
if __name__ == "__main__": main()
