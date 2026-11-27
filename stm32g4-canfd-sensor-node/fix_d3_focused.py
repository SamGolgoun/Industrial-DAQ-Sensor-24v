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
r7_nets = []
for _, _, b in fps:
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"R7\"", b):
        for pm in re.finditer(r"\(pad\b", b):
            ps = pm.start(); pe = fe(b, ps)
            if pe < 0: continue
            nm = re.search(r"\(\s*net\s+(\d+)\s+\"([^\"]+)\"\s*\)", b[ps:pe+1])
            if nm and int(nm.group(1)) > 0 and nm.group(2).strip():
                r7_nets.append((int(nm.group(1)), nm.group(2).strip()))
pad1_tgt = r7_nets[0] if r7_nets else None
gnd_id = name_to_id.get("GND") or next((v for k, v in name_to_id.items() if "GND" in k.upper()), None)
pad2_tgt = (gnd_id, id_to_name[gnd_id]) if gnd_id else None
found_d3 = False; updated_d3 = False
for st, en, b in reversed(fps):
    if re.search(r"\(\s*property\s+\"Reference\"\s+\"D3\"", b):
        found_d3 = True
        if pad1_tgt or pad2_tgt:
            counter = 0
            def repl_pad(pblk):
                global counter
                counter += 1
                tgt = pad1_tgt if counter == 1 else pad2_tgt
                if not tgt: return pblk
                nt = f"(net {tgt[0]} \"{tgt[1]}\")"
                if re.search(r"\(\s*net\s+\d+\s+\"[^\"]+\"\s*\)", pblk):
                    return re.sub(r"\(\s*net\s+\d+\s+\"[^\"]+\"\s*\)", nt, pblk, count=1)
                return pblk
            parts = []; pos = 0
            for pm in re.finditer(r"\(pad\b", b):
                ps = pm.start(); pe = fe(b, ps)
                if pe < 0 or ps < pos: continue
                parts.append(b[pos:ps]); parts.append(repl_pad(b[ps:pe+1])); pos = pe + 1
            parts.append(b[pos:])
            nb = "".join(parts)
            if nb != b:
                txt = txt[:st] + nb + txt[en:]; updated_d3 = True
        break
assert txt.count("(") == txt.count(")"), "Unbalanced output PCB"
p.write_text(txt, encoding="utf-8")
print(f"D3_FOCUSED_OK | found_d3={found_d3} | updated_d3={updated_d3} | pad1={pad1_tgt} | pad2={pad2_tgt} | balanced={txt.count(chr(40))==txt.count(chr(41))}")
