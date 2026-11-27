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
def parse_fp_full(st, en, b):
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
    for m in re.finditer(r"\(pad\s+\"([^\"]*)\"", b):
        ps = m.start()
        pe = find_end(b, ps)
        if pe < 0: continue
        pblk = b[ps:pe+1]
        pm = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", pblk)
        nm = re.search(r"\(\s*net\s+\d+\s+\"([^\"]*)\"", pblk)
        if pm:
            pads.append((m.group(1), float(pm.group(1)), float(pm.group(2)), nm.group(1).strip() if nm else ""))
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
            tot += min(math.hypot(px - qx, py - qy) for qx, qy in peers)
    return tot
def main():
    txt = PCB_PATH.read_text(encoding="utf-8")
    fps = [parse_fp_full(s, e, b) for s, e, b in iter_fps(txt)]
    by_ref = {f["ref"]: f for f in fps if f["ref"]}
    protected = {"H1","H2","H3","H4","FID1","FID2","FID3","J2","J4","U1","Y1"}
    rotated = []
    for f in fps:
        r = f["ref"]
        if not r or r in protected or not r.startswith(("C", "R")):
            continue
        cur = float(f["deg"] % 360.0)
        best_d = cur
        best_sc = score_angle(f, cur, fps)
        for cand in (0.0, 90.0, 180.0, 270.0):
            sc = score_angle(f, cand, fps)
            if sc + 0.01 < best_sc:
                best_sc = sc
                best_d = cand
        if abs(best_d - cur) > 0.01:
            rotated.append((r, int(cur), int(best_d)))
            f["deg"] = best_d
    edits = []
    for f in fps:
        if f["at_sp"]:
            sa, ea = f["at_sp"]
            d_val = float(f["deg"] % 360.0)
            d_str = int(d_val) if d_val.is_integer() else round(d_val, 3)
            na = f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f} {d_str})" if d_str != 0 else f"(at {f[chr(120)]:.3f} {f[chr(121)]:.3f})"
            edits.append((f["st"], f["en"], f["b"][:sa] + na + f["b"][ea:]))
    for s, e, nb in sorted(edits, key=lambda item: item[0], reverse=True):
        txt = txt[:s] + nb + txt[e:]
    diff = txt.count("(") - txt.count(")")
    if diff > 0: txt = txt.rstrip() + "\n" + (")" * diff) + "\n"
    PCB_PATH.write_text(txt, encoding="utf-8")
    c12_deg = by_ref["C12"]["deg"] % 360.0 if "C12" in by_ref else None
    print(f"ORIENT_FIXED_OK | rotated={len(rotated)} | C12_deg={c12_deg} | balanced={txt.count(chr(40)) == txt.count(chr(41))}")
    for item in rotated[:30]:
        print(f"  {item[0]}: rot {item[1]} -> {item[2]}")
if __name__ == "__main__": main()
