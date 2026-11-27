import json, pathlib, re, shutil, subprocess, tempfile
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
pro = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pro")
bk_dir = pathlib.Path("backups")
assert pcb.exists(), "Missing PCB file"
bks = sorted(bk_dir.glob("*.kicad_pcb*"), key=lambda x: x.stat().st_mtime, reverse=True) if bk_dir.exists() else []
prev = bks[0] if bks else None
def stats(txt):
    return {
        "balanced": txt.count("(") == txt.count(")"),
        "pads": len(re.findall(r"\(pad\b", txt)),
        "segments": len(re.findall(r"\(segment\b", txt)),
        "vias": len(re.findall(r"\(via\b", txt)),
        "zones": len(re.findall(r"\(zone\b", txt)),
        "unrouted_rats": len(re.findall(r"\(ratsnest\b", txt)),
    }
def find_cli():
    for c in ["/tmp/.mount_kicadext/usr/bin/kicad-cli", shutil.which("kicad-cli")]:
        if c and pathlib.Path(c).exists(): return c
    return None
def run_cli_drc(cli, target_pcb):
    if not cli: return None
    with tempfile.TemporaryDirectory(prefix="kicad_delta_") as td:
        tp = pathlib.Path(td) / target_pcb.name
        shutil.copy2(target_pcb, tp)
        if pro.exists(): shutil.copy2(pro, pathlib.Path(td) / pro.name)
        out = pathlib.Path(td) / "drc.json"
        subprocess.run([cli, "pcb", "drc", "--severity-all", "--format", "json", "--output", str(out), str(tp)], check=False, timeout=120, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not out.exists(): return None
        data = json.loads(out.read_text(encoding="utf-8", errors="ignore"))
        viols = data.get("violations", [])
        errs = sum(1 for v in viols if str(v.get("severity", "")).lower() == "error")
        warns = sum(1 for v in viols if str(v.get("severity", "")).lower() == "warning")
        unconn = len(data.get("unconnected_items", []))
        return {"errors": errs, "warnings": warns, "unconnected": unconn, "total_viols": len(viols)}
cur_txt = pcb.read_text(encoding="utf-8", errors="ignore")
cur_s = stats(cur_txt)
prev_s = stats(prev.read_text(encoding="utf-8", errors="ignore")) if prev else None
cli = find_cli()
cur_drc = run_cli_drc(cli, pcb)
prev_drc = run_cli_drc(cli, prev) if (prev and cli) else None
print(f"RULES_DELTA_OK | prev_backup={prev.name if prev else None} | balanced={cur_s[chr(98)+chr(97)+chr(108)+chr(97)+chr(110)+chr(99)+chr(101)+chr(100)]}")
if prev_s:
    print(f"  SEGMENTS : {prev_s[chr(115)+chr(101)+chr(103)+chr(109)+chr(101)+chr(110)+chr(116)+chr(115)]} -> {cur_s[chr(115)+chr(101)+chr(103)+chr(109)+chr(101)+chr(110)+chr(116)+chr(115)]} (delta={cur_s[chr(115)+chr(101)+chr(103)+chr(109)+chr(101)+chr(110)+chr(116)+chr(115)] - prev_s[chr(115)+chr(101)+chr(103)+chr(109)+chr(101)+chr(110)+chr(116)+chr(115)]:+d})")
    print(f"  VIAS     : {prev_s[chr(118)+chr(105)+chr(97)+chr(115)]} -> {cur_s[chr(118)+chr(105)+chr(97)+chr(115)]} (delta={cur_s[chr(118)+chr(105)+chr(97)+chr(115)] - prev_s[chr(118)+chr(105)+chr(97)+chr(115)]:+d})")
    print(f"  ZONES    : {prev_s[chr(122)+chr(111)+chr(110)+chr(101)+chr(115)]} -> {cur_s[chr(122)+chr(111)+chr(110)+chr(101)+chr(115)]} (delta={cur_s[chr(122)+chr(111)+chr(110)+chr(101)+chr(115)] - prev_s[chr(122)+chr(111)+chr(110)+chr(101)+chr(115)]:+d})")
if cur_drc is not None:
    if prev_drc is not None:
        print(f"  DRC_ERRS : {prev_drc[chr(101)+chr(114)+chr(114)+chr(111)+chr(114)+chr(115)]} -> {cur_drc[chr(101)+chr(114)+chr(114)+chr(111)+chr(114)+chr(115)]} (delta={cur_drc[chr(101)+chr(114)+chr(114)+chr(111)+chr(114)+chr(115)] - prev_drc[chr(101)+chr(114)+chr(114)+chr(111)+chr(114)+chr(115)]:+d})")
        print(f"  DRC_WARNS: {prev_drc[chr(119)+chr(97)+chr(114)+chr(110)+chr(105)+chr(110)+chr(103)+chr(115)]} -> {cur_drc[chr(119)+chr(97)+chr(114)+chr(110)+chr(105)+chr(110)+chr(103)+chr(115)]} (delta={cur_drc[chr(119)+chr(97)+chr(114)+chr(110)+chr(105)+chr(110)+chr(103)+chr(115)] - prev_drc[chr(119)+chr(97)+chr(114)+chr(110)+chr(105)+chr(110)+chr(103)+chr(115)]:+d})")
        print(f"  UNCONN   : {prev_drc[chr(117)+chr(110)+chr(99)+chr(111)+chr(110)+chr(110)+chr(101)+chr(99)+chr(116)+chr(101)+chr(100)]} -> {cur_drc[chr(117)+chr(110)+chr(99)+chr(111)+chr(110)+chr(110)+chr(101)+chr(99)+chr(116)+chr(101)+chr(100)]} (delta={cur_drc[chr(117)+chr(110)+chr(99)+chr(111)+chr(110)+chr(110)+chr(101)+chr(99)+chr(116)+chr(101)+chr(100)] - prev_drc[chr(117)+chr(110)+chr(99)+chr(111)+chr(110)+chr(110)+chr(101)+chr(99)+chr(116)+chr(101)+chr(100)]:+d})")
    else:
        print(f"  DRC_CUR  : errors={cur_drc[chr(101)+chr(114)+chr(114)+chr(111)+chr(114)+chr(115)]} | warnings={cur_drc[chr(119)+chr(97)+chr(114)+chr(110)+chr(105)+chr(110)+chr(103)+chr(115)]} | unconnected={cur_drc[chr(117)+chr(110)+chr(99)+chr(111)+chr(110)+chr(110)+chr(101)+chr(99)+chr(116)+chr(101)+chr(100)]}")
else:
    print("  CLI_DRC  : kicad-cli not found in container/path; run Design Rules Checker in KiCad or run_drc_check.py")
