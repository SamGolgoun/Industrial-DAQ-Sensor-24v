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
def parse_pads_and_at(st, en, b):
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
    pads = []
    for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"[^\n]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*?\(net\s+\d+\s+\"([^\"]*)\"\)", b):
        pads.append((pm.group(1), float(pm.group(2)), float(pm.group(3)), pm.group(4).strip()))
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp, "pads": pads}
def rot_pt(px, py, deg):
    r = math.radians(deg)
    return px * math.cos(r) - py * math.sin(r), px * math.sin(r) + py * math.cos(r)
def score_angle(fp, deg, all_fps):
    if not fp["pads"]: return 0.0
    tot = 0.0
    for _, lx, ly, net in fp["pads"]:
        if not net or "GND" in net.upper(): continue
        wx, wy = rot_pt(lx, ly, deg)
        px, py = fp["x"] + wx, fp["y"] + wy
        peers = []
        for o in all_fps:
            if o["ref"] == fp["ref"]: continue
            for _, olx, oly, onet in o["pads"]:
                if onet == net:
                    owx, owy = rot_pt(olx, oly, o["deg"])
                    peers.append((o["x"] + owx, o["y"] + owy))
        if peers:
            nearest = min(math.hypot(px - qx, py - qy) for qx, qy in peers)
            tot += nearest
    return tot
def main():
    txt = PCB_PATH.read_text(encoding="utf-8")
    fps = [parse_pads_and_at(s, e, b) for s, e, b in iter_fps(txt)]
    protected = {"H1","H2","H3","H4","FID1","FID2","FID3","J2","J4","U1","Y1"}
    updated_refs = []
    for f in fps:
        r = f["ref"]
        if not r or r in protected or not r.startswith(("C", "R")):
            continue
        best_d = f["deg"]
        best_sc = score_angle(f, best_d, fps)
        for cand in (0.0, 90.0, 180.0, 270.0):
            sc = score_angle(f, cand, fps)
            if sc + 0.05 < best_sc:
                best_sc = sc
                best_d = cand
        if abs(best_d - f["deg"]) > 0.01:
            updated_refs.append((r, int(f["deg"]), int(best_d)))
            f["deg"] = best_d
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
    print(f"ALIGN_PADS_OK | rotated={len(updated_refs)} | balanced={txt.count(chr(40)) == txt.count(chr(41))}")
    for item in updated_refs[:30]:
        print(f"  {item[0]}: rot {item[1]} -> {item[2]}")
if __name__ == "__main__": main()
