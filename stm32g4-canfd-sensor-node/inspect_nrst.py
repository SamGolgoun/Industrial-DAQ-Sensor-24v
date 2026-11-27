import pathlib, re
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
er = fe(txt, 0); i = 1; hits = []
while i < er:
    if txt[i] == "(":
        e = fe(txt, i)
        if e < 0: break
        b = txt[i:e+1]
        if b.startswith("(footprint ") and "NRST" in b.upper():
            rm = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", b)
            ref = rm.group(1) if rm else "?"
            pads = []
            for pm in re.finditer(r"\(pad\b", b):
                ps = pm.start(); pe = fe(b, ps)
                if pe < 0: continue
                pblk = b[ps:pe+1]
                if "NRST" in pblk.upper():
                    pnum_m = re.match(r"\(\s*pad\s+\"?([^\"\s)]+)\"?", pblk)
                    pnet_m = re.search(r"\(\s*net\s+(\d+)\s+\"([^\"]*)\"", pblk)
                    pads.append((pnum_m.group(1) if pnum_m else "?", pnet_m.group(1) if pnet_m else "0", pnet_m.group(2) if pnet_m else ""))
            hits.append((ref, pads))
        i = e + 1
    else: i += 1
print(f"NRST_INSPECT_OK | balanced={txt.count(chr(40))==txt.count(chr(41))} | count={len(hits)}")
for ref, pads in hits:
    print(f"  ref={ref} | pads={pads}")
