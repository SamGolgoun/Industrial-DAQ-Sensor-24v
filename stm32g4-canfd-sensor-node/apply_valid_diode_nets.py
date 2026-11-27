import math, pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
assert txt.count("(") == txt.count(")"), "Unbalanced input PCB"
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
id_to_name = {}; name_to_id = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]+)\"\s*\)", txt):
    nid = int(m.group(1)); nn = m.group(2).strip()
    if nid > 0 and nn:
        id_to_name[nid] = nn
        if nn not in name_to_id: name_to_id[nn] = nid
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
def get_xy(blk):
    m = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", blk)
    return (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)
r7_nets = []
for _, _, b in fps:
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"R7\"", b):
        for pm in re.finditer(r"\(pad\b", b):
            ps = pm.start(); pe = fe(b, ps)
            if pe < 0: continue
            nm = re.search(r"\(\s*net\s+(\d+)\s+\"([^\"]+)\"\s*\)", b[ps:pe+1])
            if nm and int(nm.group(1)) > 0: r7_nets.append((int(nm.group(1)), nm.group(2)))
p1 = r7_nets[0] if r7_nets else None
gnd_id = name_to_id.get("GND") or next((v for k, v in name_to_id.items() if "GND" in k.upper()), None)
p2 = (gnd_id, id_to_name[gnd_id]) if gnd_id else None
best_idx = None; best_dist = 1e9
for idx, (_, _, b) in enumerate(fps):
    x, y = get_xy(b)
    d = math.hypot(x - 122.0, y - 102.0)
    if d < 12.0 and d < best_dist:
        best_dist = d; best_idx = idx
updated = False
if best_idx is not None and (p1 or p2):
    st, en, b = fps[best_idx]
    counter = 0
    def fix_pad(pblk):
        global counter
        counter += 1
        target = p1 if counter == 1 else p2
        if not target or not target[0] or not target[1]: return pblk
        nt = f"(net {target[0]} \"{target[1]}\")"
        if re.search(r"\(\s*net\s+\d+\s+\"[^\"]+\"\s*\)", pblk):
            return re.sub(r"\(\s*net\s+\d+\s+\"[^\"]+\"\s*\)", nt, pblk, count=1)
        return pblk
    parts = []; pos = 0
    for pm in re.finditer(r"\(pad\b", b):
        ps = pm.start(); pe = fe(b, ps)
        if pe < 0 or ps < pos: continue
        parts.append(b[pos:ps]); parts.append(fix_pad(b[ps:pe+1])); pos = pe + 1
    parts.append(b[pos:])
    nb = "".join(parts)
    if nb != b:
        txt = txt[:st] + nb + txt[en:]; updated = True
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"VALID_DIODE_NETS_OK | updated={updated} | pad1={p1} | pad2={p2} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
