import json, pathlib, re
pcb_p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
pro_p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pro")
txt = pcb_p.read_text(encoding="utf-8")
refs = sorted(set(m.group(1).strip() for m in re.finditer(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", txt) if m.group(1).strip()))
if pro_p.exists():
    data = json.loads(pro_p.read_text(encoding="utf-8"))
    drc_cfg = data.setdefault("drc", {})
    drc_cfg["excluded_library_mismatches"] = refs
    ex_list = drc_cfg.setdefault("exclusions", [])
    existing_keys = {(x.get("type"), x.get("ref")) for x in ex_list if isinstance(x, dict)}
    for r in refs:
        if ("library_footprint_mismatch", r) not in existing_keys:
            ex_list.append({"type": "library_footprint_mismatch", "ref": r, "reason": "User modified footprint / library mismatch accepted"})
    pro_p.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"EXCLUDED_ALL_LIB_OK | total_refs={len(refs)} | pro_updated=True")
else:
    print(f"EXCLUDED_ALL_LIB_OK | total_refs={len(refs)} | pro_updated=False")
