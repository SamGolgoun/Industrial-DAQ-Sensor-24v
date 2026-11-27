import glob
import json
import math
import pathlib
import re
import shutil
import subprocess
import sys

BOARD_ORIG = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
BOARD_WORK = pathlib.Path("stm32g4-canfd-sensor-node_v2.kicad_pcb")
BACKUP_DIR = pathlib.Path("backups")
DRC_OUT = pathlib.Path("/tmp/drc_terminal_v2.json")


def find_kicad_cli():
    clis = sorted(glob.glob("/tmp/.mount_kicad*/usr/bin/kicad-cli"))
    if clis:
        return clis[-1]
    for cand in ["/usr/bin/kicad-cli", "/usr/local/bin/kicad-cli"]:
        if pathlib.Path(cand).is_file():
            return cand
    return ""


def ensure_unlocked_and_backup():
    locks = list(pathlib.Path(".").glob("~*.kicad_pcb.lck"))
    if locks:
        print("ERROR: KiCad lock file exists. Close KiCad first:", [str(p) for p in locks])
        sys.exit(2)
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    if BOARD_ORIG.is_file():
        stamp = subprocess.check_output(["date", "+%Y%m%d_%H%M%S"], text=True).strip()
        bak = BACKUP_DIR / f"stm32g4-canfd-sensor-node_untouched_{stamp}.kicad_pcb"
        if not list(BACKUP_DIR.glob("stm32g4-canfd-sensor-node_untouched_*.kicad_pcb")):
            shutil.copy2(BOARD_ORIG, bak)
            print("Backup created:", bak)
    if not BOARD_WORK.is_file() and BOARD_ORIG.is_file():
        shutil.copy2(BOARD_ORIG, BOARD_WORK)
        print("Working copy created:", BOARD_WORK)


def tokenize_sexp(text: str):
    tokens = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
        elif c in ("(", ")"):
            tokens.append(c)
            i += 1
        elif c == '"':
            j = i + 1
            buf = ['"']
            while j < n:
                if text[j] == "\\" and j + 1 < n:
                    buf.append(text[j : j + 2])
                    j += 2
                elif text[j] == '"':
                    buf.append('"')
                    j += 1
                    break
                else:
                    buf.append(text[j])
                    j += 1
            tokens.append("".join(buf))
            i = j
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in ("(", ")"):
                j += 1
            tokens.append(text[i:j])
            i = j
    return tokens


def parse_sexp(tokens):
    def helper(idx):
        assert tokens[idx] == "("
        node = []
        idx += 1
        while idx < len(tokens) and tokens[idx] != ")":
            if tokens[idx] == "(":
                child, idx = helper(idx)
                node.append(child)
            else:
                node.append(tokens[idx])
                idx += 1
        return node, idx + 1

    root, _ = helper(0)
    return root


def format_sexp(node, indent=0):
    if isinstance(node, str):
        return node
    if not node:
        return "()"
    # Keep compact nodes on one line
    simple = all(not isinstance(x, list) for x in node)
    rendered = " ".join(format_sexp(x, indent + 1) for x in node)
    if simple and len(rendered) < 110:
        return f"({rendered})"
    pad = "  " * indent
    inner = "\n".join(f"{pad}  {format_sexp(x, indent + 1)}" for x in node)
    return f"({node[0]}\n{inner}\n{pad})"


def get_fp_ref(fp):
    for item in fp[1:]:
        if isinstance(item, list) and len(item) >= 3 and item[0] == "property" and item[1].strip('"') == "Reference":
            return item[2].strip('"')
    return ""


def set_fp_pose(fp, x, y, angle=0.0):
    updated_at = False
    for idx, item in enumerate(fp[1:], start=1):
        if isinstance(item, list) and item and item[0] == "at":
            fp[idx] = ["at", f"{x:.4f}", f"{y:.4f}", f"{angle:.1f}"] if angle else ["at", f"{x:.4f}", f"{y:.4f}"]
            updated_at = True
            break
    if not updated_at:
        fp.insert(1, ["at", f"{x:.4f}", f"{y:.4f}"])


def run_drc_and_verify():
    kcli = find_kicad_cli()
    if not kcli:
        print("WARNING: kicad-cli not found in /tmp/.mount_kicad* or PATH")
        return
    cmd = [
        kcli,
        "pcb",
        "drc",
        "--refill-zones",
        "--save-board",
        "--severity-all",
        "--format",
        "json",
        "--output",
        str(DRC_OUT),
        str(BOARD_WORK),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    print("kicad-cli drc exit:", res.returncode)
    if DRC_OUT.is_file():
        data = json.loads(DRC_OUT.read_text())
        unconn = len(data.get("unconnected_items", []))
        errs = sum(1 for v in data.get("violations", []) if v.get("severity") == "error")
        print(f"DRC Summary -> Unconnected: {unconn}, Errors: {errs}")
    verifier = pathlib.Path("scripts/verify_kicad_board.py")
    if verifier.is_file():
        vr = subprocess.run([sys.executable, str(verifier)], capture_output=True, text=True, timeout=120)
        print("Verifier exit:", vr.returncode)
        if vr.stdout.strip():
            print(vr.stdout.strip())


def phase1_place(root):
    # Board outline assumed around 80x60 mm (e.g., X: 100..180, Y: 60..120 or existing Edge.Cuts)
    placements = {
        "U2": (132.0, 90.0, 0.0),
        "Y1": (121.5, 84.5, 0.0),   # Close to U2 oscillator pins with space around for GND guard vias
        "C18": (121.0, 81.5, 0.0),
        "C15": (121.0, 87.5, 0.0),
        "SW1": (143.0, 77.0, 0.0),
        "C17": (146.5, 77.0, 0.0),  # Right next to SW1 / NRST for noise suppression
        "U3": (116.0, 73.0, 0.0),   # CAN transceiver near connector
        "J1": (108.0, 73.0, 0.0),
        "U4": (152.0, 95.0, 0.0),   # Power converter / sensor block
        "L1": (158.0, 95.0, 0.0),
    }
    moved = 0
    for item in root[1:]:
        if isinstance(item, list) and item and item[0] == "footprint":
            ref = get_fp_ref(item)
            if ref in placements:
                x, y, ang = placements[ref]
                set_fp_pose(item, x, y, ang)
                moved += 1
    print(f"Phase 1 placement updated {moved} key footprints (U2, Y1 guard-ready, decoupling/noise caps, power/CAN).")


def phase2_route_summary(root):
    vias = [x for x in root[1:] if isinstance(x, list) and x and x[0] == "via"]
    segs = [x for x in root[1:] if isinstance(x, list) and x and x[0] == "segment"]
    layers_used = sorted({
        seg_layer
        for s in segs
        for sub in s[1:]
        if isinstance(sub, list) and sub and sub[0] == "layer"
        for seg_layer in [sub[1].strip('"')]
    })
    print(f"Phase 2 board inventory -> Vias: {len(vias)}, Track segments: {len(segs)}, Layers with tracks: {layers_used}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    ensure_unlocked_and_backup()
    if not BOARD_WORK.is_file():
        print("ERROR: Working PCB not found:", BOARD_WORK)
        sys.exit(1)
    tokens = tokenize_sexp(BOARD_WORK.read_text())
    root = parse_sexp(tokens)
    if mode in ("all", "phase1"):
        phase1_place(root)
    if mode in ("all", "phase2"):
        phase2_route_summary(root)
    BOARD_WORK.write_text(format_sexp(root) + "\n")
    print("Saved:", BOARD_WORK)
    run_drc_and_verify()


if __name__ == "__main__":
    main()
