import pathlib, re
p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = p.read_text(encoding="utf-8")
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
for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"\s*\)", txt):
    nid = int(m.group(1)); nname = m.group(2)
    id_to_name[nid] = nname
    if nname not in name_to_id:
        name_to_id[nname] = nid
def fix_bare_net(m):
    nid = int(m.group(1))
    nname = id_to_name.get(nid, "")
    return f"(net {nid} \"{nname}\")"
txt = re.sub(r"\(\s*net\s+(\d+)\s*\)", fix_bare_net, txt)
er = fe(txt, 0); i = 1; fps = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint "): fps.append((i, e+1, b))
        i = e + 1
    else: i += 1
def fp_pad_nets(ref):
    out = []
    for _, _, b in fps:
        if re.search(r"\(\s*property\s+\"Reference\"\s+\"" + re.escape(ref) + r"\"", b):
            for pm in re.finditer(r"\(pad\b[^\n]*\)", b):
                nm = re.search(r"\(net\s+(\d+)\s+\"([^\"]*)\"", pm.group(0))
                if nm: out.append((int(nm.group(1)), nm.group(2)))
    return out
r1_nets = fp_pad_nets("R1"); r8_nets = fp_pad_nets("R8")
shared = [nid for nid, _ in r1_nets if nid > 0 and nid in {x[0] for x in r8_nets}]
if shared:
    sig_nid = shared[0]
elif r1_nets and r1_nets[0][0] > 0:
    sig_nid = r1_nets[0][0]
elif r8_nets and r8_nets[0][0] > 0:
    sig_nid = r8_nets[0][0]
else:
    sig_nid = next((k for k in id_to_name if k > 0), 0)
sig_name = id_to_name.get(sig_nid, "")
gnd_nid = name_to_id.get("GND", next((v for k, v in name_to_id.items() if "GND" in k.upper()), 0))
gnd_name = id_to_name.get(gnd_nid, "GND")
pad_idx = 0
def repl_pad(pm):
    global pad_idx
    pad_idx += 1
    nid, nname = (sig_nid, sig_name) if pad_idx == 1 else (gnd_nid, gnd_name)
    s = pm.group(0)
    ntok = f"(net {nid} \"{nname}\")"
    if re.search(r"\(net\s+\d+\s+\"[^\"]*\"\)", s):
        return re.sub(r"\(net\s+\d+\s+\"[^\"]*\"\)", ntok, s, count=1)
    return s[:-1] + " " + ntok + ")"
updated_d3 = False
for st, en, b in reversed(fps):
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"D3\"", b):
        pad_idx = 0
        nb = re.sub(r"\(pad\b[^\n]*\)", repl_pad, b)
        if nb != b:
            txt = txt[:st] + nb + txt[en:]
            updated_d3 = True
        break
bare_left = len(re.findall(r"\(\s*net\s+\d+\s*\)", txt))
assert bare_left == 0 and txt.count("(") == txt.count(")")
p.write_text(txt, encoding="utf-8")
print(f"D3_NETNAME_OK | updated={updated_d3} | bare_left={bare_left} | sig=({sig_nid},{sig_name}) | gnd=({gnd_nid},{gnd_name}) | balanced={txt.count(chr(40))==txt.count(chr(41))}")
