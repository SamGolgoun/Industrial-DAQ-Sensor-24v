import math, pathlib, re, sys
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
target_w = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4
txt = p.read_text(encoding="utf-8")
def bal(s):
    d = q = e = 0
    for c in s:
        if q:
            if e: e = 0
            elif c == chr(92): e = 1
            elif c == chr(34): q = 0
        else:
            if c == chr(34): q = 1
            elif c == "(": d += 1
            elif c == ")":
                d -= 1
                if d < 0: return False
    return d == 0 and not q
assert bal(txt), "Unbalanced PCB"
def fe(s, idx):
    d = q = e = 0
    for i in range(idx, len(s)):
        c = s[i]
        if e: e = 0
        elif c == chr(92): e = 1
        elif c == chr(34): q ^= 1
        elif not q:
            d += (c == "(") - (c == ")")
            if d == 0: return i
    return -1
net_map = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]+)\"\s*\)", txt):
    nid = int(m.group(1)); nn = m.group(2).strip()
    if nid > 0 and nn and nn not in net_map: net_map[nn] = nid
nrst_id = net_map.get("NRST")
def rot(lx, ly, deg):
    r = math.radians(deg)
    return lx * math.cos(r) - ly * math.sin(r), lx * math.sin(r) + ly * math.cos(r)
targets = {"SW1": "2", "C17": "2", "U2": "7", "J2": "5"}
pins = []
for m in re.finditer(r"\(footprint\b", txt):
    fs = m.start(); fe_i = fe(txt, fs)
    if fe_i < 0: continue
    blk = txt[fs:fe_i+1]
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", blk)
    ref = rm.group(1).strip() if rm else ""
    if ref not in targets: continue
    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)", blk)
    fx = float(am.group(1)) if am else 0.0
    fy = float(am.group(2)) if am else 0.0
    fd = float(am.group(3)) if (am and am.group(3)) else 0.0
    for pm in re.finditer(r"\(pad\s+\"([^\"]*)\"", blk):
        if pm.group(1).strip() != targets[ref]: continue
        ps = pm.start(); pe = fe(blk, ps)
        if pe < 0: continue
        pam = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk[ps:pe+1])
        if pam:
            wx, wy = rot(float(pam.group(1)), float(pam.group(2)), fd)
            pins.append((fx + wx, fy + wy))
changed_segs = 0
def repl_seg(m):
    global changed_segs
    sblk = m.group(0)
    if nrst_id is not None and nrst_id > 0:
        if not re.search(rf"\(\s*net\s+{nrst_id}\s*\)", sblk):
            return sblk
    else:
        near = False
        for px, py in pins:
            for sm in re.finditer(r"\((?:start|end)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)", sblk):
                if math.hypot(float(sm.group(1)) - px, float(sm.group(2)) - py) < 2.5:
                    near = True; break
            if near: break
        if not near: return sblk
    nb, n = re.subn(r"\(\s*width\s+-?\d+(?:\.\d+)?\s*\)", f"(width {target_w:.4f})", sblk, count=1)
    if n > 0 and nb != sblk:
        changed_segs += 1
    return nb
txt = re.sub(r"\(segment\b[^\n]*\)", repl_seg, txt)
assert bal(txt), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"STEP_WIDTH_OK | target_w={target_w:.4f} | updated_segments={changed_segs} | balanced={bal(txt)}")
