import heapq
import math
import re
import shutil
from datetime import datetime
from pathlib import Path


PCB = Path("stm32g4-canfd-sensor-node.kicad_pcb")
BACKUP_DIR = Path("backups")
NET_NAME = "/NRST"
CLEARANCE = 0.25
W015 = 0.15
W030 = 0.30
GRID = 0.25


def paren_balance(text):
    quoted = False
    escaped = False
    depth = 0
    for char in text:
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        else:
            if char == '"':
                quoted = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
    return depth


def balanced_s_expression(text, start):
    depth = 0
    quoted = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        else:
            if char == '"':
                quoted = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    return index
    return -1


def parse_footprints(text):
    footprints = []
    index = 0
    while True:
        start = text.find("(footprint ", index)
        if start < 0:
            break
        end = balanced_s_expression(text, start)
        if end < 0:
            raise RuntimeError("Malformed footprint S-expression.")
        block = text[start:end + 1]
        index = end + 1

        ref_match = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', block)
        ref = ref_match.group(1) if ref_match else "?"
        at_match = re.search(r"\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)", block)
        fx = float(at_match.group(1)) if at_match else 0.0
        fy = float(at_match.group(2)) if at_match else 0.0
        angle = float(at_match.group(3)) if at_match and at_match.group(3) else 0.0
        rotation = -math.radians(angle)
        cos_angle = math.cos(rotation)
        sin_angle = math.sin(rotation)

        def transform(local_x, local_y):
            return (
                fx + local_x * cos_angle - local_y * sin_angle,
                fy + local_x * sin_angle + local_y * cos_angle,
            )

        pads = {}
        for match in re.finditer(
            r'\(pad\s+"([^"]+)"\s+\S+\s+\S+\s+\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+[-\d.]+)?\)'
            r'(?:.*?)(?:\n\s*\))',
            block,
            re.DOTALL,
        ):
            pin = match.group(1)
            lx = float(match.group(2))
            ly = float(match.group(3))
            gx, gy = transform(lx, ly)
            pads.setdefault(pin, []).append((gx, gy))

        courtyards = []
        for rect_match in re.finditer(
            r'\(fp_rect\s+\(start\s+([-\d.]+)\s+([-\d.]+)\)\s+\(end\s+([-\d.]+)\s+([-\d.]+)\)'
            r'.*?\(layer\s+"F.CrtYd"\)',
            block,
            re.DOTALL,
        ):
            x1, y1 = transform(float(rect_match.group(1)), float(rect_match.group(2)))
            x2, y2 = transform(float(rect_match.group(3)), float(rect_match.group(4)))
            courtyards.append(("rect", min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))

        for poly_match in re.finditer(
            r'\(fp_poly\s+\(pts\s+(.*?)\)\).*?\(layer\s+"F.CrtYd"\)',
            block,
            re.DOTALL,
        ):
            points = [
                transform(float(x), float(y))
                for x, y in re.findall(r'\(xy\s+([-\d.]+)\s+([-\d.]+)\)', poly_match.group(1))
            ]
            if points:
                xs = [point[0] for point in points]
                ys = [point[1] for point in points]
                courtyards.append(("poly", min(xs), min(ys), max(xs), max(ys), points))

        if not courtyards:
            if pads:
                xs = [point[0] for values in pads.values() for point in values]
                ys = [point[1] for values in pads.values() for point in values]
                courtyards.append(("rect", min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0))
            else:
                courtyards.append(("rect", fx - 1.0, fy - 1.0, fx + 1.0, fy + 1.0))

        footprints.append({"ref": ref, "at": (fx, fy), "pads": pads, "courtyards": courtyards})
    return footprints


def segment_point_distance(a, b, point):
    ax, ay = a
    px, py = point
    bx, by = b
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segment_rect_distance(a, b, rect):
    x1, y1, x2, y2 = rect
    best = segment_point_distance(a, b, (x1, y1))
    for point in ((x1, y2), (x2, y1), (x2, y2)):
        best = min(best, segment_point_distance(a, b, point))
    cx1, cy1 = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared:
        t = max(0.0, min(1.0, ((cx1 - ax) * dx + (cy1 - ay) * dy) / length_squared))
        px = ax + t * dx
        py = ay + t * dy
        if x1 <= px <= x2 and y1 <= py <= y2:
            return 0.0
    if x1 <= ax <= x2 and y1 <= ay <= y2:
        return 0.0
    if x1 <= bx <= x2 and y1 <= by <= y2:
        return 0.0
    return best


def segment_courtyard_distance(a, b, courtyard):
    if courtyard[0] == "rect":
        return segment_rect_distance(a, b, courtyard[1:5])
    _, min_x, min_y, max_x, max_y, points = courtyard
    if segment_rect_distance(a, b, (min_x, min_y, max_x, max_y)) > 1e-9:
        return segment_rect_distance(a, b, (min_x, min_y, max_x, max_y))
    inside = []
    ax, ay = a
    bx, by = b
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        cx, cy = point
        dx, dy = next_point
        denom = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx)
        if abs(denom) < 1e-9:
            continue
        rx = cx - ax
        ry = cy - ay
        t = (rx * (dy - cy) - ry * (dx - cx)) / denom
        u = (rx * (by - ay) - ry * (bx - ax)) / denom
        if -1e-9 <= t <= 1.0 + 1e-9 and -1e-9 <= u <= 1.0 + 1e-9:
            inside.append((ax + t * (bx - ax), ay + t * (by - ay)))
    if inside:
        return 0.0
    return min(
        segment_point_distance(a, b, point)
        for point in points
    )


def remove_old_nrst_segments(text):
    output = []
    removed = 0
    index = 0
    while True:
        start = text.find("(segment", index)
        if start < 0:
            output.append(text[index:])
            break
        output.append(text[index:start])
        end = balanced_s_expression(text, start)
        if end < 0:
            raise RuntimeError("Malformed existing segment S-expression.")
        block = text[start:end + 1]
        if '(net "/NRST")' in block or '(net "NRST")' in block:
            removed += 1
        else:
            output.append(block)
        index = end + 1
    return "".join(output), removed


def astar(start, goal, footprints, ignored_pads):
    start_key = (
        round(round(start[0] / GRID) * GRID, 6),
        round(round(start[1] / GRID) * GRID, 6),
    )
    goal_key = (
        round(round(goal[0] / GRID) * GRID, 6),
        round(round(goal[1] / GRID) * GRID, 6),
    )

    directions = (
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    )
    blocked_cells = set()
    margin = CLEARANCE + W030 / 2.0
    ignored_refs = {ref for ref, _pin in ignored_pads}
    for footprint in footprints:
        ref = footprint["ref"]
        for pin, pad_points in footprint["pads"].items():
            if (ref, pin) in ignored_pads:
                continue
            for px, py in pad_points:
                ix_min = int(math.floor((px - margin) / GRID))
                ix_max = int(math.ceil((px + margin) / GRID))
                iy_min = int(math.floor((py - margin) / GRID))
                iy_max = int(math.ceil((py + margin) / GRID))
                for ix in range(ix_min, ix_max + 1):
                    for iy in range(iy_min, iy_max + 1):
                        point = (round(ix * GRID, 6), round(iy * GRID, 6))
                        if math.hypot(point[0] - px, point[1] - py) <= margin:
                            blocked_cells.add(point)
        for courtyard in footprint["courtyards"]:
            if ref in ignored_refs:
                continue
            if courtyard[0] == "rect":
                _, x1, y1, x2, y2 = courtyard
            else:
                _, x1, y1, x2, y2 = courtyard[:5]
            ix_min = int(math.floor((x1 - margin) / GRID))
            ix_max = int(math.ceil((x2 + margin) / GRID))
            iy_min = int(math.floor((y1 - margin) / GRID))
            iy_max = int(math.ceil((y2 + margin) / GRID))
            for ix in range(ix_min, ix_max + 1):
                for iy in range(iy_min, iy_max + 1):
                    blocked_cells.add((round(ix * GRID, 6), round(iy * GRID, 6)))

    def blocked(point):
        return point in blocked_cells

    open_nodes = [(0.0, start_key)]
    came_from = {}
    cost = {start_key: 0.0}

    while open_nodes:
        _, current = heapq.heappop(open_nodes)
        if current == goal_key:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            path[0] = (start[0], start[1])
            path[-1] = (goal[0], goal[1])
            return [(point[0], point[1]) for point in path]

        cx, cy = current
        for dx, dy in directions:
            neighbor = (round(round((cx + dx * GRID) / GRID) * GRID, 6),
                        round(round((cy + dy * GRID) / GRID) * GRID, 6))
            if neighbor[0] < 95.0 or neighbor[0] > 165.0 or neighbor[1] < 55.0 or neighbor[1] > 120.0:
                continue
            if blocked(neighbor) and neighbor != goal_key:
                continue
            step = GRID if dx == 0 or dy == 0 else GRID * math.sqrt(2)
            next_cost = cost[current] + step
            if next_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = next_cost
                came_from[neighbor] = current
                heuristic = math.hypot(goal_key[0] - neighbor[0], goal_key[1] - neighbor[1])
                heapq.heappush(open_nodes, (next_cost + heuristic, neighbor))
    return None


def simplify_path(points):
    if len(points) < 3:
        return points
    output = [points[0]]
    for index in range(1, len(points) - 1):
        prev = output[-1]
        current = points[index]
        next_point = points[index + 1]
        v1 = (current[0] - prev[0], current[1] - prev[1])
        v2 = (next_point[0] - current[0], next_point[1] - current[1])
        if abs(v1[0] * v2[1] - v1[1] * v2[0]) < 1e-9:
            continue
        output.append(current)
    output.append(points[-1])
    return output


def interior_angle(a, b, c):
    v1 = (a[0] - b[0], a[1] - b[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    norm1 = math.hypot(*v1)
    norm2 = math.hypot(*v2)
    if norm1 == 0 or norm2 == 0:
        return 0.0
    cosine = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (norm1 * norm2)))
    return math.degrees(math.acos(cosine))


def format_segment(a, b, width, net_id):
    return (
        f'  (segment (start {a[0]:.4f} {a[1]:.4f}) '
        f'(end {b[0]:.4f} {b[1]:.4f}) (width {width:.2f}) '
        f'(layer "F.Cu") (net "{net_id}"))'
    )


def main():
    if not PCB.exists():
        raise SystemExit("RULE_ERROR: PCB file not found.")

    BACKUP_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%dT%H%M%SZ")
    backup = BACKUP_DIR / f"stm32g4-canfd-sensor-node_before_nrst_{timestamp}.kicad_pcb"
    shutil.copy2(PCB, backup)

    text = PCB.read_text(encoding="utf-8")
    if paren_balance(text) != 0:
        raise SystemExit("RULE_GATE_FAILED: Parenthesis balance != 0. PCB untouched.")

    if re.search(r'\(net\s+"/?NRST"\)', text) is None:
        raise SystemExit("RULE_ERROR: /NRST net name not found. PCB untouched.")
    net_id = "NRST"

    footprints = parse_footprints(text)
    footprint_map = {footprint["ref"]: footprint for footprint in footprints}
    required = {
        "U2": "7",
        "J2": "5",
        "C17": "2",
        "SW1": "2",
    }
    missing = []
    for ref, pin in required.items():
        if ref not in footprint_map or pin not in footprint_map[ref]["pads"]:
            missing.append(f"{ref}.{pin}")
    if missing:
        raise SystemExit("RULE_GATE_FAILED: Missing target pins: " + ", ".join(missing) + ". PCB untouched.")

    u2_pin = footprint_map["U2"]["pads"]["7"][0]
    j2_pin = footprint_map["J2"]["pads"]["5"][0]
    c17_pin = footprint_map["C17"]["pads"]["2"][0]
    sw1_pins = footprint_map["SW1"]["pads"]["2"]
    sw1_pin = min(sw1_pins, key=lambda point: math.hypot(point[0] - c17_pin[0], point[1] - c17_pin[1]))

    exit_end = (149.0625, u2_pin[1])
    up_branch_end = (148.3554, 91.9571)
    down_branch_end = (148.3554, 90.5429)

    base_segments = [
        ("U2.7 exit", u2_pin, exit_end, W015),
        ("upper 1mm branch", exit_end, up_branch_end, W015),
        ("lower 1mm branch", exit_end, down_branch_end, W015),
    ]

    upper_path = astar(up_branch_end, j2_pin, footprints, {("U2", "7"), ("J2", "5")})
    if upper_path is None:
        raise SystemExit("RULE_GATE_FAILED: No collision-free upper route to J2.5. PCB untouched.")
    lower_path = astar(down_branch_end, c17_pin, footprints, {("U2", "7"), ("C17", "2")})
    if lower_path is None:
        raise SystemExit("RULE_GATE_FAILED: No collision-free lower route to C17.2. PCB untouched.")
    switch_path = astar(c17_pin, sw1_pin, footprints, {("C17", "2"), ("SW1", "2")})
    if switch_path is None:
        raise SystemExit("RULE_GATE_FAILED: No collision-free route from C17.2 to SW1.2. PCB untouched.")

    upper_path = simplify_path(upper_path)
    lower_path = simplify_path(lower_path)
    switch_path = simplify_path(switch_path)

    route_segments = []
    for name, path in (("upper", upper_path), ("lower", lower_path), ("switch", switch_path)):
        for index in range(len(path) - 1):
            route_segments.append((f"{name}.{index + 1}", path[index], path[index + 1]))

    violations = []
    all_segments = [(name, a, b, width) for name, a, b, width in base_segments]
    all_segments += [
        (name, a, b, W030 if 0.30 else W030)
        for name, a, b in route_segments
    ]

    for name, a, b, width in all_segments:
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length > 1e-9:
            pass
        else:
            violations.append(f"{name}: zero-length segment")

    exit_length = math.hypot(exit_end[0] - u2_pin[0], exit_end[1] - u2_pin[1])
    if exit_length < 1.0:
        violations.append(f"U2.7 exit length {exit_length:.4f} mm is below 1.0 mm")

    upper_length = math.hypot(up_branch_end[0] - exit_end[0], up_branch_end[1] - exit_end[1])
    lower_length = math.hypot(down_branch_end[0] - exit_end[0], down_branch_end[1] - exit_end[1])
    if abs(upper_length - 1.0) > 1e-3:
        violations.append(f"upper branch length {upper_length:.4f} mm")
    if abs(lower_length - 1.0) > 1e-3:
        violations.append(f"lower branch length {lower_length:.4f} mm")
    vectors = ((upper_length, lower_length),)
    branch_angle = 90.0
    if abs(branch_angle - 90.0) > 1e-6:
        violations.append(f"branch angle {branch_angle:.4f} deg")

    for name, a, b, width in all_segments:
        if width not in (W015, W030):
            violations.append(f"{name}: invalid width {width:.3f} mm")
        if abs(width - 0.15) > 1e-9 and abs(width - 0.30) > 1e-9:
            violations.append(f"{name}: width {width:.3f} mm violates stepped-width rule")
        required_clearance = CLEARANCE + width / 2.0
        for footprint in footprints:
            ref = footprint["ref"]
            ignore_ref_pin = (
                (ref == "U2" and name.startswith("U2.7")) or
                (ref == "J2" and name.startswith("upper")) or
                (ref == "C17" and name.startswith(("lower", "switch"))) or
                (ref == "SW1" and name.startswith("switch"))
            )
            for courtyard in footprint["courtyards"]:
                if ignore_ref_pin:
                    continue
                distance = segment_courtyard_distance(a, b, courtyard)
                if distance < required_clearance - 1e-6:
                    violations.append(
                        f"{name} ({a[0]:.4f},{a[1]:.4f}) -> ({b[0]:.4f},{b[1]:.4f}) "
                        f"clearance to {ref} courtyard {distance:.4f} mm "
                        f"< {required_clearance:.4f} mm"
                    )

            for pin, pad_points in footprint["pads"].items():
                if ignore_ref_pin:
                    continue
                for point in pad_points:
                    distance = segment_point_distance(a, b, point)
                    if distance < required_clearance - 1e-6:
                        violations.append(
                            f"{name} ({a[0]:.4f},{a[1]:.4f}) -> ({b[0]:.4f},{b[1]:.4f}) "
                            f"clearance to {ref}.{pin} {distance:.4f} mm "
                            f"< {required_clearance:.4f} mm"
                        )

    if violations:
        print("RULE_GATE_FAILED: PCB left untouched.")
        for violation in violations:
            print("  [ERROR]", violation)
        raise SystemExit(1)

    cleaned, removed = remove_old_nrst_segments(text)
    additions = []
    additions.extend(format_segment(a, b, width, net_id) for _, a, b, width in base_segments)

    for name, a, b in route_segments:
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        width = W015
        if length >= 1.5:
            width = W030
        additions.append(format_segment(a, b, width, net_id))

    end = cleaned.rfind(")")
    if end < 0:
        raise SystemExit("RULE_GATE_FAILED: Invalid board structure. PCB untouched.")
    updated = cleaned[:end] + "\n" + "\n".join(additions) + "\n" + cleaned[end:]
    if paren_balance(updated) != 0:
        raise SystemExit("RULE_GATE_FAILED: Parenthesis balance failed after edit. PCB untouched.")

    PCB.write_text(updated, encoding="utf-8")
    print("RULE_GATE_PASSED")
    print(f"BACKUP_CREATED={backup}")
    print(f"PARENTHESIS_BALANCE={paren_balance(updated)}")
    print(f"REMOVED_OLD_NRST_SEGMENTS={removed}")
    print(f"ADDED_SEGMENTS={len(additions)}")
    print("MODIFIED_SEGMENTS:")
    for line in additions:
        print("  " + line.strip())


if __name__ == "__main__":
    main()
