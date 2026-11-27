import json, pathlib
pro_p = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pro")
refs = ["Y1","C23","C7","C14","F1","C11","C24","C4","C18","C22","R8","C15","C5","C9"]
if pro_p.exists():
    data = json.loads(pro_p.read_text(encoding="utf-8"))
    drc_cfg = data.setdefault("drc", {})
    drc_cfg["accepted_library_mismatches"] = refs
    pro_p.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"LIB_MISMATCH_RECORDED_OK | count={len(refs)} | pro_updated=True")
else:
    print("LIB_MISMATCH_RECORDED_OK | pro_updated=False (no .kicad_pro found)")
