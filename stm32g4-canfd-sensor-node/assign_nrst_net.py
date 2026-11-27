import pathlib, re
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
net_map = {}
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt):
    nid = int(m.group(1)); nn = m.group(2).strip()
    if nid > 0 and nn and nn not in net_map:
        net_map[nn] = nid
max_id = max([0] + [int(m.group(1)) for m in re.finditer(r"\(\s*net\s+(\d+)\b", txt)])
if "NRST" in net_map:
    nrst_id = net_map["NRST"]
else:
    nrst_id = max_id + 1
    net_map["NRST"] = nrst_id
    last_net = None
    for m in re.finditer(r"\(\s*net\s+\d+\s+\"[^\"]*\"\s*\)", txt):
        last_net = m
    ins = f"\n  (net {nrst_id} \"NRST\")"
    if last_net:
        txt = txt[:last_net.end()] + ins + txt[last_net.end():]
    else:
        hm = re.search(r"\(\s*version\b[^\n]*\n", txt)
        if hm: txt = txt[:hm.end()] + ins + "\n" + txt[hm.end():]
target_refs = {"SW1", "U2", "J2", "C17"}
updated_pads = 0
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
for st, en, b in reversed(fps):
    rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
    ref = rm.group(1) if rm else ""
    if ref not in target_refs: continue
    parts = []; pos = 0; changed = False
    for pm in re.finditer(r"\(pad\b", b):
        ps = pm.start(); pe = fe(b, ps)
        if pe < 0 or ps < pos: continue
        parts.append(b[pos:ps])
        pblk = b[ps:pe+1]
        if "NRST" in pblk.upper():
            nt = f"(net {nrst_id} \"NRST\")"
            if re.search(r"\(\s*net\s+\d+\s+\"[^\"]*\"\s*\)", pblk):
                npblk = re.sub(r"\(\s*net\s+\d+\s+\"[^\"]*\"\s*\)", nt, pblk, count=1)
            else:
                npblk = pblk[:-1] + " " + nt + ")"
            if npblk != pblk:
                updated_pads += 1; changed = True
            parts.append(npblk)
        else:
            parts.append(pblk)
        pos = pe + 1
    parts.append(b[pos:])
    if changed:
        txt = txt[:st] + "".join(parts) + txt[en:]
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"ASSIGN_NRST_OK | nrst_id={nrst_id} | updated_pads={updated_pads} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
