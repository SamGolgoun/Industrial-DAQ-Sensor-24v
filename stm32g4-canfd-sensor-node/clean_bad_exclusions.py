import pathlib
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
while True:
    idx = txt.find("(drc_exclusions")
    if idx == -1: break
    end = fe(txt, idx)
    if end == -1: break
    s_cut = idx - 1 if idx > 0 and txt[idx - 1] == "\n" else idx
    e_cut = end + 1 if end + 1 < len(txt) and txt[end + 1] == "\n" else end + 1
    txt = txt[:s_cut] + txt[e_cut:]
assert txt.count("(") == txt.count(")"), "Unbalanced PCB"
p.write_text(txt, encoding="utf-8")
print("CLEAN_EXCLUSIONS_OK | has_drc_exclusions=", "drc_exclusions" in txt, "| balanced=", txt.count(chr(40)) == txt.count(chr(41)))
