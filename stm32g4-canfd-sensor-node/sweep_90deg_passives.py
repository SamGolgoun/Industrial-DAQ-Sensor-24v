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
def segments_intersect(p1, p2, q1, q2):
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    o1 = orient(p1, p2, q1); o2 = orient(p1, p2, q2)
    o3 = orient(q1, q2, p1); o4 = orient(q1, q2, p2)
    return (o1 * o2 < 0) and (o3 * o4 < 0)
def build_edges(fp, deg, all_fps):
    edges = []
    for _, lx, ly, net in fp["pads"]:
        if not net: continue
        wx, wy = rot_pt(lx, ly, deg)
        p_start = (fp["x"] + wx, fp["y"] + wy)
        peers = []
        for o in all_fps:
            if o["ref"] == fp["ref"]: continue
            for _, olx, oly, onet in o["pads"]:
                if onet == net:
                    owx, owy = rot_pt(olx, oly, o["deg"])
                    peers.append((o["x"] + owx, o["y"] + owy))
        if peers:
            p_end = min(peers, key=lambda q: math.hypot(p_start[0] - q[0], p_start[1] - q[1]))
            edges.append((net, p_start, p_end))
    return edges
def other_edges(target_ref, all_fps):
    out = []
    for o in all_fps:
        if o["ref"] == target_ref: continue
        for _, lx, ly, net in o["pads"]:
            if not net: continue
            wx, wy = rot_pt(lx, ly, o["deg"])
            a = (o["x"] + wx, o["y"] + wy)
            b = (o["x"], o["y"])
            out.append((net, a, b))
    return out
def evaluate_angle(fp, deg, all_fps, ref_others):
    edges = build_edges(fp, deg, all_fps)
    route_len = 0.0
    crossings = 0
    for net, p1, p2 in edges:
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        weight = 1.5 if "GND" not in net.upper() else 0.8
        route_len += weight * dist
        for onet, q1, q2 in ref_others:
            if onet != net and segments_intersect(p1, p2, q1, q2):
                crossings += 1
    total = route_len + 3.5 * crossings
    return round(total, 4), round(route_len, 3), crossings
def main():
    txt = PCB_PATH.read_text(encoding="utf-8")
    fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
    targets = [f for f in fps if f["ref"] and f["ref"].startswith(("C", "R", "D"))]
    report = []
    for f in targets:
        cur = float(f["deg"] % 360.0)
        others = other_edges(f["ref"], fps)
        cur_tot, cur_len, cur_crs = evaluate_angle(f, cur, fps, others)
        best_d = cur
        best_tot, best_len, best_crs = cur_tot, cur_len, cur_crs
        # Test four 90-degree steps around 360 degrees
        for step in range(4):
            cand = (cur + step * 90.0) % 360.0
            tot, rlen, crs = evaluate_angle(f, cand, fps, others)
            if tot + 0.02 < best_tot:
                best_tot, best_len, best_crs = tot, rlen, crs
                best_d = cand
        if abs(best_d - cur) > 0.01:
            report.append((f["ref"], int(cur), int(best_d), cur_len, best_len, cur_crs, best_crs))
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
    bal = (txt.count(chr(40)) == txt.count(chr(41)))
    print(f"SWEEP_4X90_DONE | evaluated={len(targets)} | changed={len(report)} | balanced={bal}")
    for r, d0, d1, l0, l1, c0, c1 in report:
        print(f"  {r}: {d0}deg -> {d1}deg | len {l0}->{l1} | crossings {c0}->{c1}")
if __name__ == "__main__": main()
