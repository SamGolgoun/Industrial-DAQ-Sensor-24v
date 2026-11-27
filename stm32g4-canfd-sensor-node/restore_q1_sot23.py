import datetime, pathlib, re, shutil
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
bk_dir = pathlib.Path("backups")
bk_dir.mkdir(exist_ok=True)
if pcb.exists():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(pcb, bk_dir / f"bk_pre_0023_restore_{stamp}.kicad_pcb")
candidates = sorted(list(bk_dir.glob("*.kicad_pcb*")) + [p for p in pathlib.Path(".").glob("*.kicad_pcb*") if p.name != pcb.name], key=lambda x: x.stat().st_mtime, reverse=True)
chosen = None
for f in candidates:
    if not f.is_file() or "bk_pre_0023_restore_" in f.name: continue
    try: t = f.read_text(encoding="utf-8", errors="ignore")
    except Exception: continue
    if t.count("(") != t.count(")"): continue
    q1_blocks = re.findall(r"\(footprint\s+\"([^\"]+)\"[\s\S]*?\(property\s+\"Reference\"\s+\"Q1\"[\s\S]*?(?=\n\s*\(footprint|\n\s*\(zone|\n\s*\))", t)
    fp_name = q1_blocks[0] if q1_blocks else ""
    if "SOT-23" in fp_name and "D_SOD-123" not in fp_name:
        chosen = f; break
if chosen:
    shutil.copy2(chosen, pcb)
    rt = pcb.read_text(encoding="utf-8", errors="ignore")
    dt_str = datetime.datetime.fromtimestamp(chosen.stat().st_mtime)
    print(f"FOUND_AND_RESTORED_OK | file={chosen} | mtime={dt_str} | balanced={rt.count(chr(40))==rt.count(chr(41))}")
else:
    print("NO_MATCHING_SOT23_BACKUP_FOUND | total_candidates=", len(candidates))
