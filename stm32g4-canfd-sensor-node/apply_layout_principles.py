import math, pathlib, re

# Apply approved placement principles to stm32g4-canfd-sensor-node.kicad_pcb
# with pcbnew closed and parentheses balanced.

PCB_PATH = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")


def find_end(s: str, idx: int) -> int:
    depth = 0
    in_str = False
    esc = False
    for i in range(idx, len(s)):
        ch = s[i]
        if esc:
            esc = False
        elif ch == chr(92):
            esc = True
        elif ch == chr(34):
            in_str = not in_str
        elif not in_str:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    return i
    return -1


def iter_footprints(s: str):
    end_root = find_end(s, 0)
    i = 1
    while i < end_root:
        if s[i] == "(":
            e = find_end(s, i)
            if e < 0:
                break
            blk = s[i : e + 1]
            if blk.startswith("(footprint "):
                yield i, e + 1, blk
            i = e + 1
        else:
            i += 1


def parse_footprint(st: int, en: int, blk: str) -> dict:
    ref_m = re.search(r"\(\s*property\s+\"Reference\"\s+\"([^\"]+)\"", blk)
    ref = ref_m.group(1).strip() if ref_m else ""
    fx = fy = fdeg = 0.0
    at_span = None
    i = 1
    depth = 1
    in_str = False
    esc = False
    while i < len(blk):
        ch = blk[i]
        if esc:
            esc = False
        elif ch == chr(92):
            esc = True
        elif ch == chr(34):
            in_str = not in_str
        elif not in_str:
            if ch == "(":
                if depth == 1 and re.match(r"^\(\s*at\b", blk[i : i + 10]):
                    ae = find_end(blk, i)
                    am = re.match(
                        r"^\(\s*at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?\s*\)$",
                        blk[i : ae + 1],
                    )
                    if am:
                        fx = float(am.group(1))
                        fy = float(am.group(2))
                        fdeg = float(am.group(3)) if am.group(3) else 0.0
                        at_span = (i, ae + 1)
                        break
                depth += 1
            elif ch == ")":
                depth -= 1
        i += 1
    is_sensitive = bool(re.search(r"ADC|ANA|VREF|DAC", blk, re.I))
    return {
        "st": st,
        "en": en,
        "blk": blk,
        "ref": ref,
        "x": fx,
        "y": fy,
        "deg": fdeg,
        "at_span": at_span,
        "sensitive": is_sensitive,
    }


def detect_board_origin(text: str) -> tuple[float, float]:
    xs = [
        float(m.group(1))
        for m in re.finditer(
            r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts",
            text,
        )
    ]
    ys = [
        float(m.group(2))
        for m in re.finditer(
            r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts",
            text,
        )
    ]
    if xs and ys:
        return min(xs), min(ys)
    return 100.0, 60.0


def layout_grid(items: list[dict], start_x: float, start_y: float, cols: int, step_x: float, step_y: float) -> None:
    for idx, item in enumerate(items):
        col = idx % cols
        row = idx // cols
        item["x"] = round(start_x + col * step_x, 3)
        item["y"] = round(start_y + row * step_y, 3)


def main() -> None:
    txt = PCB_PATH.read_text(encoding="utf-8")
    x0, y0 = detect_board_origin(txt)
    x1, y1 = x0 + 80.0, y0 + 60.0

    fps = [parse_footprint(s, e, b) for s, e, b in iter_footprints(txt)]
    by_ref = {f["ref"]: f for f in fps if f["ref"]}

    # Corner mounting holes (5.0 mm inset), fiducials inside board, connectors, and MCU+crystal
    anchors = {
        "H1": (x0 + 5.0, y0 + 5.0),
        "H2": (x1 - 5.0, y0 + 5.0),
        "H3": (x1 - 5.0, y1 - 5.0),
        "H4": (x0 + 5.0, y1 - 5.0),
        "FID1": (x0 + 13.5, y0 + 7.5),
        "FID2": (x1 - 13.5, y0 + 7.5),
        "FID3": (x1 - 13.5, y1 - 7.5),
        "J2": (x1 - 12.5, y0 + 14.5),
        "J4": (x0 + 42.0, y1 - 10.5),
        # Sensitive core: U1 and Y1 placed close (~5.5 mm) with open guard margin around them
        "U1": (x0 + 48.0, y0 + 27.5),
        "Y1": (x0 + 43.2, y0 + 24.0),
    }

    for ref, xy in anchors.items():
        if ref in by_ref:
            by_ref[ref]["x"], by_ref[ref]["y"] = xy

    noisy_power = []
    noisy_comm = []
    decoupling_caps = []
    sensitive_blocks = []
    far_resistors = []
    jumpers = []

    for fp in fps:
        ref = fp["ref"]
        if not ref or ref in anchors:
            continue
        if ref.startswith(("L", "FB", "Q", "D", "F", "SW")):
            noisy_power.append(fp)
        elif ref in {"U2", "U3", "U4", "U5"} or ref.startswith("J"):
            noisy_comm.append(fp)
        elif ref.startswith(("JP", "JMP")):
            jumpers.append(fp)
        elif ref.startswith("R"):
            far_resistors.append(fp)
        elif ref.startswith("C") and (fp["sensitive"] or len(decoupling_caps) < 8):
            decoupling_caps.append(fp)
        elif fp["sensitive"] or ref.startswith("U"):
            sensitive_blocks.append(fp)
        elif ref.startswith("C"):
            noisy_power.append(fp)
        else:
            noisy_comm.append(fp)

    # Noisy power in upper-left away from sensitive core
    layout_grid(noisy_power, x0 + 12.0, y0 + 15.0, cols=4, step_x=4.9, step_y=4.9)

    # Decoupling capacitors kept close to U1/Y1 while preserving routing & GND guard ring space
    layout_grid(decoupling_caps, x0 + 38.5, y0 + 34.0, cols=4, step_x=4.3, step_y=4.1)
    layout_grid(sensitive_blocks, x0 + 39.0, y0 + 16.5, cols=3, step_x=4.8, step_y=4.6)

    # Resistors placed farther away with generous spacing so pins remain accessible
    layout_grid(far_resistors, x0 + 13.0, y1 - 21.5, cols=6, step_x=5.4, step_y=4.8)

    # CAN/IO/jumpers in outer/lower zones inside board
    layout_grid(noisy_comm, x1 - 25.0, y0 + 21.0, cols=2, step_x=6.4, step_y=5.8)
    layout_grid(jumpers, x0 + 22.0, y1 - 11.5, cols=3, step_x=5.2, step_y=4.4)

    if "U1" in by_ref and "Y1" in by_ref:
        dist_uy = math.hypot(
            by_ref["U1"]["x"] - by_ref["Y1"]["x"],
            by_ref["U1"]["y"] - by_ref["Y1"]["y"],
        )
        assert 4.0 <= dist_uy <= 8.5, f"Unexpected U1-Y1 distance: {dist_uy}"

    if "J2" in by_ref and "U1" in by_ref:
        dist_ju = math.hypot(
            by_ref["J2"]["x"] - by_ref["U1"]["x"],
            by_ref["J2"]["y"] - by_ref["U1"]["y"],
        )
        assert dist_ju > 14.0, f"J2 too close to U1: {dist_ju}"

    edits = []
    for fp in fps:
        if fp["at_span"]:
            sa, ea = fp["at_span"]
            deg_str = int(fp["deg"]) if float(fp["deg"]).is_integer() else fp["deg"]
            new_at = (
                f"(at {fp[chr(120)]:.3f} {fp[chr(121)]:.3f} {deg_str})"
                if fp["deg"] != 0
                else f"(at {fp[chr(120)]:.3f} {fp[chr(121)]:.3f})"
            )
            edits.append((fp["st"], fp["en"], fp["blk"][:sa] + new_at + fp["blk"][ea:]))

    for s, e, new_blk in sorted(edits, key=lambda item: item[0], reverse=True):
        txt = txt[:s] + new_blk + txt[e:]

    diff = txt.count("(") - txt.count(")")
    if diff > 0:
        txt = txt.rstrip() + "\n" + (")" * diff) + "\n"

    PCB_PATH.write_text(txt, encoding="utf-8")
    print(
        f"APPLIED_PRINCIPLES_OK | moved={len(edits)} | decoup_caps={len(decoupling_caps)} "
        f"| far_res={len(far_resistors)} | noisy_pwr={len(noisy_power)} | noisy_comm={len(noisy_comm)} "
        f"| balanced={txt.count(chr(40)) == txt.count(chr(41))}"
    )


if __name__ == "__main__":
    main()
