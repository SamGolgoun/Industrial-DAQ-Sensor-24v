import datetime, pathlib, re, shutil
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
bk_dir = pathlib.Path("backups")
files = sorted(bk_dir.glob("*.kicad_pcb*"), key=lambda x: x.stat().st_mtime, reverse=True)
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
chosen = None
for f in files:
    t = f.read_text(encoding="utf-8", errors="ignore")
    if t.count("(") != t.count(")") or "(segment " in t:
        continue
    er = fe(t, 0); i = 1
    while i < er:
        if t[i] == "(":
            e = fe(t, i)
            if e < 0: break
            b = t[i:e+1]
            if b.startswith("(footprint ") and re.search(r"\(\s*property\s+\"Reference\"\s+\"Q1\"", b):
                if "SOT-23" in b and "D_SOD-123" not in b:
                    am = re.search(r"\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", b)
                    if am and abs(float(am.group(1)) - 117.5) < 3.0 and abs(float(am.group(2)) - 78.5) < 3.0:
                        chosen = f
                break
            i = e + 1
        else: i += 1
    if chosen: break
if chosen:
    shutil.copy2(chosen, pcb)
    rt = pcb.read_text(encoding="utf-8")
    dt_s = datetime.datetime.fromtimestamp(chosen.stat().st_mtime)
    print(f"EXACT_RESTORE_OK | file={chosen} | mtime={dt_s} | balanced={rt.count(chr(40))==rt.count(chr(41))}")
else:
    print("NOT_FOUND | checked_backups=", len(files))
