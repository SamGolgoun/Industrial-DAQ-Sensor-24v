import math, pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
assert txt.count("(") == txt.count(")"), "Unbalanced PCB"
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
id_to_name = {0: ""}
name_to_id = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"?([^\"\s)]*)\"?\s*\)", txt):
    nid = int(m.group(1)); nn = m.group(2)
    id_to_name[nid] = nn
    if nn and nn not in name_to_id: name_to_id[nn] = nid
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
def extract_pad_nets(blk):
    res = []
    for pm in re.finditer(r"\(pad\b[\s\S]*?\)", blk):
        ps = pm.start(); pe = fe(blk, ps)
        if pe < 0: continue
        pblk = blk[ps:pe+1]
        nm = re.search(r"\(\s*net\s+(\d+)(?:\s+\"?([^\"\s)]*)\"?)?\s*\)", pblk)
        if nm:
            nid = int(nm.group(1))
            nn = nm.group(2) if nm.group(2) else id_to_name.get(nid, "")
            res.append((nid, nn))
    return res
r7_nets = []
for _, _, b in fps:
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"R7\"", b) or re.search(r"\(\s*fp_text\s+reference\s+\"?R7\"?", b, re.I):
        r7_nets = extract_pad_nets(b)
        if r7_nets: break
if not r7_nets:
    for _, _, b in fps:
        x, y = get_xy(b)
        if math.hypot(x - 119.0, y - 103.5) < 5.0:
            r7_nets = extract_pad_nets(b)
            if r7_nets: break
p1_nid, p1_name = next(((nid, nn or id_to_name.get(nid, "")) for nid, nn in r7_nets if nid > 0), (0, ""))
p2_nid = name_to_id.get("GND", next((v for k, v in name_to_id.items() if "GND" in k.upper()), 0))
p2_name = id_to_name.get(p2_nid, "GND")
best_idx = None; best_dist = 1e9
for idx, (_, _, b) in enumerate(fps):
    x, y = get_xy(b)
    d = math.hypot(x - 122.0, y - 102.0)
    if d < 12.0 and d < best_dist:
        best_dist = d; best_idx = idx
updated = False
if best_idx is not None:
    st, en, b = fps[best_idx]
    counter = 0
    def set_net_in_pad(pblk):
        global counter
        counter += 1
        nid, nn = (p1_nid, p1_name) if counter == 1 else (p2_nid, p2_name)
        nt = f"(net {nid} \"{nn}\")" if nn else f"(net {nid})"
        if re.search(r"\(\s*net\s+\d+(?:\s+\"?[^\"\s)]*\"?)?\s*\)", pblk):
            return re.sub(r"\(\s*net\s+\d+(?:\s+\"?[^\"\s)]*\"?)?\s*\)", nt, pblk, count=1)
        return pblk[:-1] + " " + nt + ")"
    parts = []; pos = 0
    for pm in re.finditer(r"\(pad\b", b):
        ps = pm.start(); pe = fe(b, ps)
        if pe < 0 or ps < pos: continue
        parts.append(b[pos:ps])
        parts.append(set_net_in_pad(b[ps:pe+1]))
        pos = pe + 1
    parts.append(b[pos:])
    nb = "".join(parts)
    if nb != b:
        txt = txt[:st] + nb + txt[en:]
        updated = True
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"FIX_DIODE_NETS_OK | updated={updated} | dist={best_dist:.2f} | pad1=({p1_nid},{p1_name}) | pad2=({p2_nid},{p2_name}) | balanced={txt.count(chr(40))==txt.count(chr(41))}")
