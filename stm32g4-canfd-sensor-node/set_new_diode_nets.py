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
net_id_to_name = {0: ""}
net_name_to_id = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt):
    nid, nn = int(m.group(1)), m.group(2)
    net_id_to_name[nid] = nn
    if nn not in net_name_to_id: net_name_to_id[nn] = nid
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
r7_pad_nets = []
for _, _, b in fps:
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"R7\"", b):
        for pm in re.finditer(r"\(pad\b[^\n]*\)", b):
            nm = re.search(r"\(net\s+(\d+)\s+\"([^\"]*)\"", pm.group(0))
            if nm: r7_pad_nets.append((int(nm.group(1)), nm.group(2)))
p1_nid, p1_name = r7_pad_nets[0] if r7_pad_nets else (0, "")
p2_nid = net_name_to_id.get("GND", next((v for k, v in net_name_to_id.items() if "GND" in k.upper()), 0))
p2_name = net_id_to_name.get(p2_nid, "GND")
best_idx = None; best_dist = 1e9
for idx, (_, _, b) in enumerate(fps):
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"D1\"", b):
        x, y = get_xy(b)
        d = math.hypot(x - 122.0, y - 102.0)
        if d < best_dist:
            best_dist = d; best_idx = idx
updated = False
if best_idx is not None:
    st, en, b = fps[best_idx]
    counter = 0
    def repl(pm):
        global counter
        counter += 1
        nid, nn = (p1_nid, p1_name) if counter == 1 else (p2_nid, p2_name)
        nt = f"(net {nid} \"{nn}\")"
        s = pm.group(0)
        if re.search(r"\(net\s+\d+\s+\"[^\"]*\"\)", s):
            return re.sub(r"\(net\s+\d+\s+\"[^\"]*\"\)", nt, s, count=1)
        return s[:-1] + " " + nt + ")"
    nb = re.sub(r"\(pad\b[^\n]*\)", repl, b)
    if nb != b:
        txt = txt[:st] + nb + txt[en:]
        updated = True
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"NEW_DIODE_NETS_OK | updated={updated} | pad1=({p1_nid},{p1_name}) | pad2=({p2_nid},{p2_name}) | balanced={txt.count(chr(40))==txt.count(chr(41))}")
