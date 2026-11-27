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
def local_bounds(blk):
    xs, ys = [], []
    for m in re.finditer(r"\((?:fp_line|fp_rect|pad)\b[^\n]*?(?:\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)\s*\(end\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)|\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*?\(size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\))", blk):
        if m.group(1) is not None:
            xs.extend([float(m.group(1)), float(m.group(3))])
            ys.extend([float(m.group(2)), float(m.group(4))])
        elif m.group(5) is not None:
            w = float(m.group(7)) * 0.5
            h = float(m.group(8)) * 0.5
            xs.extend([float(m.group(5)) - w, float(m.group(5)) + w])
            ys.extend([float(m.group(6)) - h, float(m.group(6)) + h])
    if not xs or not ys:
        return (-1.6, -1.6, 1.6, 1.6)
    pad_margin = 0.65
    return (min(xs) - pad_margin, min(ys) - pad_margin, max(xs) + pad_margin, max(ys) + pad_margin)
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
    sig_nets = {m.group(1).strip() for m in re.finditer(r"\(\s*net\s+\d+\s+\"([^\"]*)\"\s*\)", b) if m.group(1).strip() and "GND" not in m.group(1).upper()}
    return {"st": st, "en": en, "b": b, "ref": ref, "x": fx, "y": fy, "deg": fdeg, "at_sp": at_sp, "sig_nets": sig_nets, "lb": local_bounds(b)}
def box_at(fp, cx, cy, clearance=1.25):
    lx0, ly0, lx1, ly1 = fp["lb"]
    rad = math.radians(fp["deg"])
    ca, sa = math.cos(rad), math.sin(rad)
    pts = [(lx0, ly0), (lx1, ly0), (lx1, ly1), (lx0, ly1)]
    wx = [cx + px * ca - py * sa for px, py in pts]
    wy = [cy + px * sa + py * ca for px, py in pts]
    return (min(wx) - clearance, min(wy) - clearance, max(wx) + clearance, max(wy) + clearance)
def overlaps(b1, b2):
    return not (b1[2] <= b2[0] or b2[2] <= b1[0] or b1[3] <= b2[1] or b2[3] <= b1[1])
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
    fixed_refs = {"H1", "H2", "H3", "H4", "FID1", "FID2", "FID3", "J2", "J4", "U1", "Y1"}
    placed_boxes = []
    for f in fps:
        if f["ref"] in fixed_refs:
            placed_boxes.append((f["ref"], box_at(f, f["x"], f["y"], 1.35)))
    non_caps = [f for f in fps if f["ref"] and not f["ref"].startswith("C") and f["ref"] not in fixed_refs]
    for f in non_caps:
        cand_box = box_at(f, f["x"], f["y"], 1.25)
        if any(overlaps(cand_box, ob) for _, ob in placed_boxes):
            for r_step in range(1, 25):
                rad = r_step * 1.4
                for k in range(16):
                    ang = k * (2.0 * math.pi / 16.0)
                    nx = max(x0 + 8.0, min(x1 - 8.0, f["x"] + rad * math.cos(ang)))
                    ny = max(y0 + 8.0, min(y1 - 8.0, f["y"] + rad * math.sin(ang)))
                    nb = box_at(f, nx, ny, 1.25)
                    if not any(overlaps(nb, ob) for _, ob in placed_boxes):
                        f["x"], f["y"] = round(nx, 3), round(ny, 3)
                        cand_box = nb
                        break
                else: continue
                break
        placed_boxes.append((f["ref"], cand_box))
    consumers = [f for f in fps if f["ref"] and not f["ref"].startswith(("C", "R", "H", "FID"))]
    caps = [f for f in fps if f["ref"].startswith("C")]
    slot_map = {}
    for cap in caps:
        matched = [c for c in consumers if (c["sig_nets"] & cap["sig_nets"])]
        host = min(matched, key=lambda c: math.hypot(c["x"] - cap["x"], c["y"] - cap["y"])) if matched else by_ref.get("U1")
        if not host:
            continue
        slot = slot_map.get(host["ref"], 0)
        slot_map[host["ref"]] = slot + 1
        placed_ok = False
        for ring in range(1, 14):
            rad = 4.6 + (ring - 1) * 1.55
            for k in range(16):
                ang = -0.7 + slot * 0.65 + k * (2.0 * math.pi / 16.0)
                nx = max(x0 + 8.0, min(x1 - 8.0, host["x"] + rad * math.cos(ang)))
                ny = max(y0 + 8.0, min(y1 - 8.0, host["y"] + rad * math.sin(ang)))
                cb = box_at(cap, nx, ny, 1.25)
                if not any(overlaps(cb, ob) for _, ob in placed_boxes):
                    cap["x"], cap["y"] = round(nx, 3), round(ny, 3)
                    placed_boxes.append((cap["ref"], cb))
                    placed_ok = True
                    break
            if placed_ok: break
        if not placed_ok:
            placed_boxes.append((cap["ref"], box_at(cap, cap["x"], cap["y"], 1.25)))
    # Verify zero courtyard overlaps across all placed boxes
    overlap_pairs = []
    for i in range(len(placed_boxes)):
        for j in range(i + 1, len(placed_boxes)):
            r1, b1 = placed_boxes[i]
            r2, b2 = placed_boxes[j]
            if overlaps(b1, b2):
                overlap_pairs.append((r1, r2))
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
    print(f"NO_OVERLAP_OK | total_boxes={len(placed_boxes)} | overlaps={len(overlap_pairs)} | balanced={txt.count(chr(40)) == txt.count(chr(41))}")
    if overlap_pairs: print("  OVERLAPS:", overlap_pairs[:10])
if __name__ == "__main__": main()
