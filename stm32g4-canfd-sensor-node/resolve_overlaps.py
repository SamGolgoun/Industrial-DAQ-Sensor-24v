import math
import pathlib
import re

pcb_path = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
txt = pcb_path.read_text(encoding="utf-8")


def is_balanced(s: str) -> bool:
    depth = 0
    quoted = False
    escaped = False
    for ch in s:
        if quoted:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                quoted = False
        else:
            if ch == '"':
                quoted = True
            elif ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth < 0:
                    return False
    return depth == 0 and not quoted


assert is_balanced(txt), "Input PCB has unbalanced parentheses"


def find_end(s: str, idx: int) -> int:
    depth = 0
    quoted = False
    escaped = False
    for i in range(idx, len(s)):
        ch = s[i]
        if escaped:
            escaped = False
        elif ch == "\\":
            escaped = True
        elif ch == '"':
            quoted = not quoted
        elif not quoted:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    return i
    return -1


def iter_fps(s: str):
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


def local_extents(blk: str) -> tuple[float, float, float, float]:
    xs, ys = [], []
    for m in re.finditer(
        r"\(pad\b[^\n]*?\(at\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+-?\d+(?:\.\d+)?)?\)[^\n]*?\(size\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)",
        blk,
    ):
        px, py = float(m.group(1)), float(m.group(2))
        hw, hh = float(m.group(3)) * 0.5, float(m.group(4)) * 0.5
        xs.extend([px - hw, px + hw])
        ys.extend([py - hh, py + hh])

    for m in re.finditer(
        r"\((?:fp_line|fp_rect)\b[^\n]*?\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)\s*\(end\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*?(?:CrtYd|SilkS|Fab)",
        blk,
    ):
        xs.extend([float(m.group(1)), float(m.group(3))])
        ys.extend([float(m.group(2)), float(m.group(4))])

    if not xs or not ys:
        return (-1.6, -1.6, 1.6, 1.6)

    margin = 0.50
    return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)


def parse_fp(st: int, en: int, blk: str) -> dict:
    rm = re.search(r'\(\s*property\s+"Reference"\s+"([^"]+)"', blk)
    ref = rm.group(1).strip() if rm else ""
    fx = fy = fdeg = 0.0
    at_span = None
    i = 1
    depth = 1
    quoted = False
    escaped = False
    while i < len(blk):
        ch = blk[i]
        if escaped:
            escaped = False
        elif ch == "\\":
            escaped = True
        elif ch == '"':
            quoted = not quoted
        elif not quoted:
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

    return {
        "st": st,
        "en": en,
        "blk": blk,
        "ref": ref,
        "x": fx,
        "y": fy,
        "deg": fdeg,
        "at_span": at_span,
        "ext": local_extents(blk),
    }


def world_rect(fp: dict, x: float, y: float, min_clearance: float = 1.0) -> tuple[float, float, float, float]:
    lx0, ly0, lx1, ly1 = fp["ext"]
    rad = math.radians(fp["deg"])
    ca, sa = math.cos(rad), math.sin(rad)
    corners = [(lx0, ly0), (lx1, ly0), (lx1, ly1), (lx0, ly1)]
    wx = [x + px * ca - py * sa for px, py in corners]
    wy = [y + px * sa + py * ca for px, py in corners]
    pad = 0.5 * min_clearance
    return (min(wx) - pad, min(wy) - pad, max(wx) + pad, max(wy) + pad)


def rects_overlap(a, b) -> bool:
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


xs = [
    float(m.group(1))
    for m in re.finditer(
        r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts",
        txt,
    )
]
ys = [
    float(m.group(2))
    for m in re.finditer(
        r"\((?:gr_line|gr_arc)\s+\(start\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\)[^\n]*Edge\.Cuts",
        txt,
    )
]
x0 = min(xs) if xs else 100.0
y0 = min(ys) if ys else 60.0
x1, y1 = x0 + 80.0, y0 + 60.0

fps = [parse_fp(s, e, b) for s, e, b in iter_fps(txt)]
MIN_CLEARANCE = 1.0
locked_refs = {"H1", "H2", "H3", "H4", "FID1", "FID2", "FID3"}

before_collisions = []
for i in range(len(fps)):
    for j in range(i + 1, len(fps)):
        if rects_overlap(
            world_rect(fps[i], fps[i]["x"], fps[i]["y"], MIN_CLEARANCE),
            world_rect(fps[j], fps[j]["x"], fps[j]["y"], MIN_CLEARANCE),
        ):
            before_collisions.append((fps[i]["ref"], fps[j]["ref"]))

placed = []
for f in fps:
    if f["ref"] in locked_refs:
        placed.append((f, world_rect(f, f["x"], f["y"], MIN_CLEARANCE)))

moved = []
for f in fps:
    if f["ref"] in locked_refs:
        continue
    ox, oy = f["x"], f["y"]
    cur_rect = world_rect(f, ox, oy, MIN_CLEARANCE)
    if any(rects_overlap(cur_rect, pr) for _, pr in placed):
        resolved = False
        for step in range(1, 50):
            dist = step * 0.65
            n_pts = max(16, step * 8)
            for k in range(n_pts):
                ang = 2.0 * math.pi * k / n_pts
                nx = max(x0 + 7.0, min(x1 - 7.0, ox + dist * math.cos(ang)))
                ny = max(y0 + 7.0, min(y1 - 7.0, oy + dist * math.sin(ang)))
                cand = world_rect(f, nx, ny, MIN_CLEARANCE)
                if not any(rects_overlap(cand, pr) for _, pr in placed):
                    f["x"], f["y"] = round(nx, 3), round(ny, 3)
                    cur_rect = cand
                    moved.append((f["ref"], round(ox, 2), round(oy, 2), f["x"], f["y"]))
                    resolved = True
                    break
            if resolved:
                break
    placed.append((f, cur_rect))

after_collisions = []
for i in range(len(placed)):
    for j in range(i + 1, len(placed)):
        if rects_overlap(placed[i][1], placed[j][1]):
            after_collisions.append((placed[i][0]["ref"], placed[j][0]["ref"]))

edits = []
for f in fps:
    if f["at_span"]:
        sa, ea = f["at_span"]
        dv = float(f["deg"] % 360.0)
        ds = int(dv) if dv.is_integer() else round(dv, 3)
        na = f"(at {f['x']:.3f} {f['y']:.3f} {ds})" if ds != 0 else f"(at {f['x']:.3f} {f['y']:.3f})"
        edits.append((f["st"], f["en"], f["blk"][:sa] + na + f["blk"][ea:]))

for s, e, nb in sorted(edits, key=lambda item: item[0], reverse=True):
    txt = txt[:s] + nb + txt[e:]

assert is_balanced(txt), "Patched PCB has unbalanced parentheses"
pcb_path.write_text(txt, encoding="utf-8")

print(
    f"PAD_COURTYARD_RESOLVED | clearance={MIN_CLEARANCE}mm | before_collisions={len(before_collisions)} "
    f"| moved={len(moved)} | after_collisions={len(after_collisions)} | balanced={is_balanced(pcb_path.read_text(encoding='utf-8'))}"
)
for item in moved[:30]:
    print(f"  MOVED {item[0]}: ({item[1]},{item[2]}) -> ({item[3]},{item[4]})")
if after_collisions:
    print("  REMAINING_COLLISIONS:", after_collisions[:20])
