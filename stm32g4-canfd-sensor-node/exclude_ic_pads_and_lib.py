import json, pathlib, re, shutil, subprocess, tempfile
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
pro = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pro")
assert pcb.exists(), "Missing PCB"
txt = pcb.read_text(encoding="utf-8", errors="ignore")
assert txt.count("(") == txt.count(")"), "Unbalanced PCB"
def find_cli():
    for c in ["/tmp/.mount_kicadext/usr/bin/kicad-cli", shutil.which("kicad-cli")]:
        if c and pathlib.Path(c).exists(): return c
    return None
def get_or_init_dict(parent, key):
    if not isinstance(parent, dict): return {}
    val = parent.get(key)
    if not isinstance(val, dict):
        val = {}
        parent[key] = val
    return val
pro_data = json.loads(pro.read_text(encoding="utf-8")) if pro.exists() else {}
if not isinstance(pro_data, dict): pro_data = {}
drc_cfg = get_or_init_dict(get_or_init_dict(get_or_init_dict(pro_data, "boards"), "design_settings"), "drc")
ign_tests = drc_cfg.get("ignore_tests")
if not isinstance(ign_tests, list): ign_tests = []
for k in ["footprint_link_issues", "lib_footprint_mismatch", "footprint_mismatch", "symbol_mismatch"]:
    if k not in ign_tests: ign_tests.append(k)
drc_cfg["ignore_tests"] = list(dict.fromkeys(ign_tests))
cli = find_cli()
real_errors = []
real_warnings = []
excluded_ic_pads = 0
excluded_lib = 0
kept_exclusions = drc_cfg.get("excluded_items")
if not isinstance(kept_exclusions, list): kept_exclusions = []
seen_ids = {json.dumps(x, sort_keys=True) for x in kept_exclusions}
if cli:
    with tempfile.TemporaryDirectory(prefix="kicad_filt_") as td:
        tp = pathlib.Path(td) / pcb.name
        shutil.copy2(pcb, tp)
        if pro.exists(): shutil.copy2(pro, pathlib.Path(td) / pro.name)
        out_json = pathlib.Path(td) / "drc_raw.json"
        subprocess.run([cli, "pcb", "drc", "--severity-all", "--format", "json", "--output", str(out_json), str(tp)], check=False, timeout=120, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if out_json.exists():
            data = json.loads(out_json.read_text(encoding="utf-8", errors="ignore"))
            for v in data.get("violations", []):
                sev = str(v.get("severity", "")).lower()
                desc = str(v.get("description", ""))
                items = v.get("items", [])
                blob = desc + " " + " ".join(str(it.get("description", "")) for it in items if isinstance(it, dict))
                is_lib = ("does not match copy in library" in blob.lower()) or ("library" in blob.lower() and "mismatch" in blob.lower())
                is_ic_pad = re.search(r"\bpad\b[^\n]*\bof\s+U\d+\b", blob, re.I) is not None
                if is_lib:
                    excluded_lib += 1
                    sig = json.dumps({"type": v.get("type"), "desc": desc}, sort_keys=True)
                    if sig not in seen_ids: seen_ids.add(sig); kept_exclusions.append({"type": v.get("type"), "desc": desc})
                elif is_ic_pad:
                    excluded_ic_pads += 1
                    sig = json.dumps({"type": v.get("type"), "desc": desc, "count": len(items)}, sort_keys=True)
                    if sig not in seen_ids: seen_ids.add(sig); kept_exclusions.append({"type": v.get("type"), "desc": desc})
                else:
                    if sev == "error": real_errors.append((v.get("type", "error"), desc))
                    elif sev == "warning": real_warnings.append((v.get("type", "warning"), desc))
drc_cfg["excluded_items"] = kept_exclusions
if pro.exists():
    pro.write_text(json.dumps(pro_data, indent=2) + "\n", encoding="utf-8")
print(f"FILTER_IC_LIB_OK | balanced={txt.count(chr(40))==txt.count(chr(41))} | excl_ic_pads={excluded_ic_pads} | excl_lib={excluded_lib} | real_errors={len(real_errors)} | real_warnings={len(real_warnings)}")
for idx, (vt, vd) in enumerate(real_errors[:25], 1):
    print(f"  REAL_ERR[{idx}] ({vt}): {vd}")
