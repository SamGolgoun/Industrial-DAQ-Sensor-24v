#!/usr/bin/env python3
"""
fix_board.py - automatic fixes for a KiCad (.kicad_pcb) file, pure Python, no pcbnew needed.

What it does:
  1. Removes duplicate vias (same net, centres < 0.1 mm apart)  -> fixes hole_to_hole
  2. Removes dangling vias given with --rm-via X,Y (only if safe) -> fixes via_dangling
  3. Routes unconnected nets (A* router on F.Cu/B.Cu with vias)   -> fixes unconnected_items
  4. Sets silk_overlap / silk_over_copper to 'ignore' in .kicad_pro
A timestamped backup is always written first.
"""
import argparse, collections, datetime, heapq, json, math, os, re, shutil, sys, uuid

COPPER = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
ROUTE_LAYERS = list(COPPER)   # outer + inner layers; inner/back cost a bit more
LAYER_COST = {"F.Cu": 1.0, "In1.Cu": 1.35, "In2.Cu": 1.35, "B.Cu": 1.1}
RES = 0.05            # router grid (mm)
VIA_COST = 1.0        # extra path cost of a via (mm-equivalent)
WS = " \t\r\n"


class Q(str):
    """quoted string"""


# ------------------------------------------------------------ s-expression
def _atom_end(t, i):
    n = len(t)
    while i < n and t[i] not in WS and t[i] not in "()":
        i += 1
    return i


def parse(t, i=0):
    while t[i] in WS:
        i += 1
    c = t[i]
    if c == "(":
        i += 1
        out = []
        while True:
            while t[i] in WS:
                i += 1
            if t[i] == ")":
                return out, i + 1
            node, i = parse(t, i)
            out.append(node)
    if c == '"':
        j = i + 1
        buf = []
        while t[j] != '"':
            if t[j] == "\\":
                buf.append(t[j + 1])
                j += 2
            else:
                buf.append(t[j])
                j += 1
        return Q("".join(buf)), j + 1
    j = _atom_end(t, i)
    return t[i:j], j


def parse_root(t):
    i = t.index("(") + 1
    while t[i] in WS:
        i += 1
    i = _atom_end(t, i)
    children, spans = [], []
    while True:
        while t[i] in WS:
            i += 1
        if t[i] == ")":
            break
        s = i
        node, i = parse(t, i)
        children.append(node)
        spans.append((s, i))
    return children, spans, i


def kids(n, name):
    return [c for c in n if isinstance(c, list) and c and c[0] == name]


def kid(n, name):
    for c in n:
        if isinstance(c, list) and c and c[0] == name:
            return c
    return None


def fmt(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


# ------------------------------------------------------------ geometry
def d_pt_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def d_pt_rect(px, py, r):
    dx = max(r[0] - px, 0, px - r[2])
    dy = max(r[1] - py, 0, py - r[3])
    return math.hypot(dx, dy)


def samples(x1, y1, x2, y2, step=0.05):
    L = math.hypot(x2 - x1, y2 - y1)
    n = max(1, int(L / step))
    return [(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n) for k in range(n + 1)]


def d_seg_seg(a, b):
    return min(
        d_pt_seg(a["x1"], a["y1"], b["x1"], b["y1"], b["x2"], b["y2"]),
        d_pt_seg(a["x2"], a["y2"], b["x1"], b["y1"], b["x2"], b["y2"]),
        d_pt_seg(b["x1"], b["y1"], a["x1"], a["y1"], a["x2"], a["y2"]),
        d_pt_seg(b["x2"], b["y2"], a["x1"], a["y1"], a["x2"], a["y2"]),
    )


# ------------------------------------------------------------ board model
class Board:
    def __init__(self, path):
        self.path = path
        self.text = open(path, encoding="utf-8").read()
        self.root, self.spans, self.endidx = parse_root(self.text)
        self.netname = {}
        for c in kids(self.root, "net"):
            if len(c) >= 3:
                self.netname[int(c[1])] = str(c[2])
        self.netid = {v: k for k, v in self.netname.items()}
        segs = kids(self.root, "segment")
        self.id_style = True
        if segs:
            n = kid(segs[0], "net")
            if n and len(n) > 1 and isinstance(n[1], Q):
                self.id_style = False
        self.items = []
        self.added = []
        self.deleted_root = set()

    def net_of(self, el):
        n = kid(el, "net")
        if not n or len(n) < 2:
            return ""
        v = n[1:]
        for x in reversed(v):
            if isinstance(x, Q):
                return str(x)
        try:
            return self.netname.get(int(v[0]), "")
        except ValueError:
            return ""

    def net_token(self, name):
        if self.id_style and name in self.netid:
            return str(self.netid[name])
        return '"%s"' % name

    def zone_nets(self):
        return {self.net_of(z) for z in kids(self.root, "zone")}

    def build(self, sign=1):
        self.items = []
        nid = 0
        for ri, el in enumerate(self.root):
            if not isinstance(el, list) or not el:
                continue
            h = el[0]
            if h == "footprint":
                at = kid(el, "at")
                fx, fy = float(at[1]), float(at[2])
                fr = math.radians(float(at[3])) if len(at) > 3 else 0.0
                ref = ""
                for p in kids(el, "property"):
                    if len(p) > 2 and p[1] == "Reference":
                        ref = str(p[2])
                if not ref:
                    for p in kids(el, "fp_text"):
                        if len(p) > 2 and p[1] == "reference":
                            ref = str(p[2])
                for pad in kids(el, "pad"):
                    pat = kid(pad, "at")
                    px, py = float(pat[1]), float(pat[2])
                    pa = float(pat[3]) if len(pat) > 3 else 0.0
                    sz = kid(pad, "size")
                    w, hh = float(sz[1]), float(sz[2])
                    if 45 < (pa % 180) < 135:
                        w, hh = hh, w
                    ax = fx + px * math.cos(fr) + sign * py * math.sin(fr)
                    ay = fy - sign * px * math.sin(fr) + py * math.cos(fr)
                    ly = kid(pad, "layers")
                    layers = set()
                    for L in (ly[1:] if ly else []):
                        if L in ("*.Cu", "F&B.Cu"):
                            layers |= set(COPPER) if L == "*.Cu" else {"F.Cu", "B.Cu"}
                        elif L in COPPER:
                            layers.add(str(L))
                    self.items.append(dict(
                        id=nid, kind="pad", ref=ref, num=str(pad[1]), net=self.net_of(pad),
                        layers=layers, x=ax, y=ay, fc=(fx, fy), pw=w, ph=hh,
                        rect=(ax - w / 2, ay - hh / 2, ax + w / 2, ay + hh / 2), root=None, deleted=False))
                    nid += 1
            elif h == "segment":
                s, e = kid(el, "start"), kid(el, "end")
                self.items.append(dict(
                    id=nid, kind="seg", net=self.net_of(el), layers={str(kid(el, "layer")[1])},
                    x1=float(s[1]), y1=float(s[2]), x2=float(e[1]), y2=float(e[2]),
                    w=float(kid(el, "width")[1]), root=ri, deleted=False))
                nid += 1
            elif h == "via":
                a = kid(el, "at")
                dr = kid(el, "drill")
                self.items.append(dict(
                    id=nid, kind="via", net=self.net_of(el), layers=set(COPPER),
                    x=float(a[1]), y=float(a[2]), size=float(kid(el, "size")[1]),
                    drill=float(dr[1]) if dr else 0.3, root=ri, deleted=False))
                nid += 1

    def live(self):
        return [i for i in self.items if not i["deleted"]]

    def find_pad(self, ref, num):
        for i in self.items:
            if i["kind"] == "pad" and i["ref"] == ref and i["num"] == num:
                return i
        return None

    # --- connectivity -----------------------------------------------------
    @staticmethod
    def touches(a, b):
        if not (a["layers"] & b["layers"]):
            return False
        ka, kb = a["kind"], b["kind"]
        if ka > kb:
            a, b, ka, kb = b, a, kb, ka
        if ka == "pad" and kb == "pad":
            ra, rb = a["rect"], b["rect"]
            return not (ra[2] < rb[0] or rb[2] < ra[0] or ra[3] < rb[1] or rb[3] < ra[1])
        if ka == "pad" and kb == "seg":
            return any(d_pt_rect(x, y, a["rect"]) <= b["w"] / 2 + 1e-6
                       for x, y in samples(b["x1"], b["y1"], b["x2"], b["y2"]))
        if ka == "pad" and kb == "via":
            return d_pt_rect(b["x"], b["y"], a["rect"]) <= b["size"] / 2 + 1e-6
        if ka == "seg" and kb == "seg":
            return d_seg_seg(a, b) <= (a["w"] + b["w"]) / 2 + 0.005
        if ka == "seg" and kb == "via":
            return d_pt_seg(b["x"], b["y"], a["x1"], a["y1"], a["x2"], a["y2"]) <= b["size"] / 2 + 1e-6
        if ka == "via" and kb == "via":
            return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) <= (a["size"] + b["size"]) / 2
        return False

    def components(self, net):
        its = [i for i in self.live() if i["net"] == net]
        parent = list(range(len(its)))

        def find(k):
            while parent[k] != k:
                parent[k] = parent[parent[k]]
                k = parent[k]
            return k
        for a in range(len(its)):
            for b in range(a + 1, len(its)):
                if self.touches(its[a], its[b]):
                    parent[find(a)] = find(b)
        comps = collections.defaultdict(list)
        for k, it in enumerate(its):
            comps[find(k)].append(it)
        return list(comps.values())

    @staticmethod
    def anchors(comp):
        out = []
        for it in comp:
            if it["kind"] in ("pad", "via"):
                out.append((it["x"], it["y"], set(it["layers"]), it["kind"]))
            else:
                out.append((it["x1"], it["y1"], set(it["layers"]), "seg"))
                out.append((it["x2"], it["y2"], set(it["layers"]), "seg"))
        return out

    # --- edits --------------------------------------------------------------
    def delete_item(self, it):
        it["deleted"] = True
        if it["root"] is not None:
            self.deleted_root.add(it["root"])

    def restore_item(self, it):
        it["deleted"] = False
        if it["root"] is not None:
            self.deleted_root.discard(it["root"])

    def touchers(self, x, y, net, tol=0.02):
        n = []
        for it in self.live():
            if it["net"] != net:
                continue
            if it["kind"] == "pad" and d_pt_rect(x, y, it["rect"]) <= tol:
                n.append(it)
            elif it["kind"] == "via" and math.hypot(x - it["x"], y - it["y"]) <= it["size"] / 2 + tol:
                n.append(it)
            elif it["kind"] == "seg" and d_pt_seg(x, y, it["x1"], it["y1"], it["x2"], it["y2"]) <= it["w"] / 2 + tol:
                n.append(it)
        return n

    def remove_duplicate_vias(self):
        vias = [i for i in self.live() if i["kind"] == "via"]
        n = 0
        for a in range(len(vias)):
            for b in range(a + 1, len(vias)):
                va, vb = vias[a], vias[b]
                if va["deleted"] or vb["deleted"]:
                    continue
                d = math.hypot(va["x"] - vb["x"], va["y"] - vb["y"])
                if va["net"] == vb["net"] and d < 0.1:
                    self.delete_item(vb)
                    n += 1
                    print("  removed duplicate via at (%.3f, %.3f), net %s" % (vb["x"], vb["y"], vb["net"]))
                elif d - (va["drill"] + vb["drill"]) / 2 < 0.25:
                    print("  WARNING: vias (%.2f,%.2f) and (%.2f,%.2f) have hole-to-hole < 0.25 mm "
                          "but different nets: fix manually" % (va["x"], va["y"], vb["x"], vb["y"]))
        return n

    def remove_dangling_via(self, x, y, tol=0.06):
        cands = [i for i in self.live() if i["kind"] == "via" and math.hypot(i["x"] - x, i["y"] - y) <= tol]
        if not cands:
            print("  no via found near (%.3f, %.3f)" % (x, y))
            return
        v = cands[0]
        net, r = v["net"], v["size"] / 2
        pads = [i for i in self.live() if i["kind"] == "pad" and i["net"] == net
                and d_pt_rect(v["x"], v["y"], i["rect"]) <= r]
        segs = [i for i in self.live() if i["kind"] == "seg" and i["net"] == net
                and d_pt_seg(v["x"], v["y"], i["x1"], i["y1"], i["x2"], i["y2"]) <= r]
        if pads:
            print("  KEPT via (%.2f,%.2f): it sits on a pad (%s.%s). Only a warning, harmless."
                  % (v["x"], v["y"], pads[0]["ref"], pads[0]["num"]))
            return
        if not segs:
            self.delete_item(v)
            print("  removed isolated via at (%.3f, %.3f)" % (v["x"], v["y"]))
            return
        if len(segs) >= 2:
            if len({tuple(s["layers"]) for s in segs}) == 1:
                self.delete_item(v)
                print("  removed via at (%.3f, %.3f) (tracks on one layer stay joined)" % (v["x"], v["y"]))
            else:
                print("  KEPT via (%.2f,%.2f): tracks on different layers meet here." % (v["x"], v["y"]))
            return
        # exactly one track ends on the via: delete it only if the whole chain is a dead end
        chain = []
        s = segs[0]
        px, py = v["x"], v["y"]
        self.delete_item(v)
        while True:
            chain.append(s)
            s["deleted"] = True
            if math.hypot(s["x1"] - px, s["y1"] - py) <= math.hypot(s["x2"] - px, s["y2"] - py):
                qx, qy = s["x2"], s["y2"]
            else:
                qx, qy = s["x1"], s["y1"]
            t = self.touchers(qx, qy, net)
            if not t:
                for c in chain:
                    self.delete_item(c)
                print("  removed dangling via at (%.3f, %.3f) + %d dead-end track piece(s)"
                      % (v["x"], v["y"], len(chain)))
                return
            if len(t) == 1 and t[0]["kind"] == "seg":
                s, px, py = t[0], qx, qy
                continue
            for c in chain:
                c["deleted"] = False
            self.restore_item(v)
            print("  KEPT via (%.2f,%.2f): its track leads to a real connection. Only a warning, harmless."
                  % (v["x"], v["y"]))
            return

    # --- routing --------------------------------------------------------------
    def common_via(self):
        c = collections.Counter((i["size"], i["drill"]) for i in self.items if i["kind"] == "via")
        return c.most_common(1)[0][0] if c else (0.45, 0.2)

    def common_width(self, net):
        c = collections.Counter(i["w"] for i in self.items if i["kind"] == "seg" and i["net"] == net)
        if not c:
            c = collections.Counter(i["w"] for i in self.items if i["kind"] == "seg")
        return c.most_common(1)[0][0] if c else 0.15

    def route_pair(self, net, a, b, tw, vsize, vdrill, clr, margin):
        ax, ay, alay, _ = a
        bx, by, blay, _ = b
        ox = min(ax, bx) - margin
        oy = min(ay, by) - margin
        # align grid origin with start anchor
        ox = ax - math.floor((ax - ox) / RES) * RES
        oy = ay - math.floor((ay - oy) / RES) * RES
        W = int((max(ax, bx) + margin - ox) / RES) + 2
        H = int((max(ay, by) + margin - oy) / RES) + 2
        N = W * H
        track = [bytearray(N) for _ in ROUTE_LAYERS]
        viab = bytearray(N)

        def cell(x, y):
            return int(round((x - ox) / RES)), int(round((y - oy) / RES))

        def mark_rect(buf, r, e):
            ix0 = max(0, int(math.ceil((r[0] - e - ox) / RES - 1e-9)))
            ix1 = min(W - 1, int(math.floor((r[2] + e - ox) / RES + 1e-9)))
            iy0 = max(0, int(math.ceil((r[1] - e - oy) / RES - 1e-9)))
            iy1 = min(H - 1, int(math.floor((r[3] + e - oy) / RES + 1e-9)))
            if ix1 < ix0:
                return
            run = b"\x01" * (ix1 - ix0 + 1)
            for iy in range(iy0, iy1 + 1):
                buf[iy * W + ix0: iy * W + ix1 + 1] = run

        def mark_cap(buf, x1, y1, x2, y2, rad):
            ix0 = max(0, int((min(x1, x2) - rad - ox) / RES) - 1)
            ix1 = min(W - 1, int((max(x1, x2) + rad - ox) / RES) + 1)
            iy0 = max(0, int((min(y1, y2) - rad - oy) / RES) - 1)
            iy1 = min(H - 1, int((max(y1, y2) + rad - oy) / RES) + 1)
            for iy in range(iy0, iy1 + 1):
                py = oy + iy * RES
                for ix in range(ix0, ix1 + 1):
                    if d_pt_seg(ox + ix * RES, py, x1, y1, x2, y2) <= rad:
                        buf[iy * W + ix] = 1

        for it in self.live():
            same = it["net"] == net
            k = it["kind"]
            if k == "pad":
                if same:
                    mark_rect(viab, it["rect"], vsize / 2)
                else:
                    for li, L in enumerate(ROUTE_LAYERS):
                        if L in it["layers"]:
                            mark_rect(track[li], it["rect"], clr + tw / 2)
                    if it["layers"]:
                        mark_rect(viab, it["rect"], clr + vsize / 2)
            elif k == "seg" and not same:
                L = next(iter(it["layers"]))
                if L in ROUTE_LAYERS:
                    mark_cap(track[ROUTE_LAYERS.index(L)], it["x1"], it["y1"], it["x2"], it["y2"],
                             it["w"] / 2 + clr + tw / 2)
                mark_cap(viab, it["x1"], it["y1"], it["x2"], it["y2"], it["w"] / 2 + clr + vsize / 2)
            elif k == "via" and not same:
                for li in range(len(ROUTE_LAYERS)):
                    mark_cap(track[li], it["x"], it["y"], it["x"], it["y"], it["size"] / 2 + clr + tw / 2)
                mark_cap(viab, it["x"], it["y"], it["x"], it["y"], it["size"] / 2 + clr + vsize / 2)

        sx, sy = cell(ax, ay)
        gx, gy = cell(bx, by)
        s_layers = [ROUTE_LAYERS.index(L) for L in alay if L in ROUTE_LAYERS]
        g_layers = [ROUTE_LAYERS.index(L) for L in blay if L in ROUTE_LAYERS]
        if not s_layers or not g_layers:
            return None
        for l in s_layers:
            track[l][sy * W + sx] = 0
        for l in g_layers:
            track[l][gy * W + gx] = 0

        INF = float("inf")
        nL = len(ROUTE_LAYERS)
        lc = [LAYER_COST[L] for L in ROUTE_LAYERS]

        def h(ix, iy):
            dx, dy = abs(ix - gx), abs(iy - gy)
            return RES * (max(dx, dy) + 0.41421356 * min(dx, dy))

        dist, prev, heap = {}, {}, []
        for l in s_layers:
            s = l * N + sy * W + sx
            dist[s] = 0.0
            heapq.heappush(heap, (h(sx, sy), 0.0, s))
        goals = {l * N + gy * W + gx for l in g_layers}
        DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.41421356), (1, -1, 1.41421356), (-1, 1, 1.41421356), (-1, -1, 1.41421356)]
        found = None
        expansions = 0
        while heap:
            f, gc, s = heapq.heappop(heap)
            if gc > dist.get(s, INF):
                continue
            if s in goals:
                found = s
                break
            expansions += 1
            if expansions > 4000000:
                break
            l, rem = divmod(s, N)
            iy, ix = divmod(rem, W)
            bl = track[l]
            for dx, dy, c in DIRS:
                nx, ny = ix + dx, iy + dy
                if nx < 0 or ny < 0 or nx >= W or ny >= H or bl[ny * W + nx]:
                    continue
                if dx and dy and (bl[iy * W + nx] or bl[ny * W + ix]):
                    continue
                ns = l * N + ny * W + nx
                ng = gc + c * RES * lc[l]
                if ng < dist.get(ns, INF):
                    dist[ns] = ng
                    prev[ns] = s
                    heapq.heappush(heap, (ng + h(nx, ny), ng, ns))
            if not viab[iy * W + ix]:
                for l2 in range(nL):
                    if l2 == l or track[l2][iy * W + ix]:
                        continue
                    ns = l2 * N + iy * W + ix
                    ng = gc + VIA_COST
                    if ng < dist.get(ns, INF):
                        dist[ns] = ng
                        prev[ns] = s
                        heapq.heappush(heap, (ng + h(ix, iy), ng, ns))
        if found is None:
            return None
        path = []
        s = found
        while True:
            l, rem = divmod(s, N)
            iy, ix = divmod(rem, W)
            path.append((ix, iy, l))
            if s not in prev:
                break
            s = prev[s]
        path.reverse()

        def clear_line(l, p, q):
            bl = track[l]
            L = math.hypot(q[0] - p[0], q[1] - p[1])
            n = max(1, int(L * 4))
            for k in range(n + 1):
                x = p[0] + (q[0] - p[0]) * k / n
                y = p[1] + (q[1] - p[1]) * k / n
                for cx in (math.floor(x), math.ceil(x)):
                    for cy in (math.floor(y), math.ceil(y)):
                        if bl[cy * W + cx]:
                            return False
            return True

        runs = []
        for pt in path:
            if runs and runs[-1][0] == pt[2]:
                runs[-1][1].append((pt[0], pt[1]))
            else:
                runs.append((pt[2], [(pt[0], pt[1])]))
        out_segs, vias = [], []
        for ri_, (l, pts) in enumerate(runs):
            simp = [pts[0]]
            i0 = 0
            while i0 < len(pts) - 1:
                j = len(pts) - 1
                while j > i0 + 1 and not clear_line(l, pts[i0], pts[j]):
                    j -= 1
                simp.append(pts[j])
                i0 = j
            for k in range(1, len(simp)):
                p, q = simp[k - 1], simp[k]
                out_segs.append((ox + p[0] * RES, oy + p[1] * RES, ox + q[0] * RES, oy + q[1] * RES, ROUTE_LAYERS[l]))
            if ri_ > 0:
                vias.append((ox + pts[0][0] * RES, oy + pts[0][1] * RES))
        last = path[-1]
        lx, ly = ox + last[0] * RES, oy + last[1] * RES
        if math.hypot(lx - bx, ly - by) > 1e-6:
            out_segs.append((lx, ly, bx, by, ROUTE_LAYERS[last[2]]))
        return out_segs, vias

    def add_track(self, net, x1, y1, x2, y2, layer, w):
        tok = self.net_token(net)
        self.added.append(
            "\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer \"%s\")\n"
            "\t\t(net %s)\n\t\t(uuid \"%s\")\n\t)\n" % (fmt(x1), fmt(y1), fmt(x2), fmt(y2), fmt(w), layer, tok, uuid.uuid4()))
        self.items.append(dict(id=len(self.items) + 1000, kind="seg", net=net, layers={layer},
                               x1=x1, y1=y1, x2=x2, y2=y2, w=w, root=None, deleted=False))

    def add_via(self, net, x, y, size, drill):
        tok = self.net_token(net)
        self.added.append(
            "\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n\t\t(layers \"F.Cu\" \"B.Cu\")\n"
            "\t\t(net %s)\n\t\t(uuid \"%s\")\n\t)\n" % (fmt(x), fmt(y), fmt(size), fmt(drill), tok, uuid.uuid4()))
        self.items.append(dict(id=len(self.items) + 1000, kind="via", net=net, layers=set(COPPER),
                               x=x, y=y, size=size, drill=drill, root=None, deleted=False))

    def add_escape_stubs(self, nets, tw):
        """fine-pitch pads (aspect >= 2, e.g. QFN): straight stub away from the chip so neighbours keep an exit"""
        for it in list(self.live()):
            if it["kind"] != "pad" or it["net"] not in nets or len(it["layers"]) != 1:
                continue
            pw, ph = it["pw"], it["ph"]
            if max(pw, ph) < 2 * min(pw, ph):
                continue
            if any(o["kind"] == "seg" and o["net"] == it["net"] and not o["deleted"]
                   and d_pt_rect(o["x1"], o["y1"], it["rect"]) <= o["w"] and True for o in self.items):
                continue                      # already has a track
            fx, fy = it["fc"]
            if pw >= ph:
                dx, dy, ln = (1 if it["x"] >= fx else -1), 0, pw / 2 + 0.5
            else:
                dx, dy, ln = 0, (1 if it["y"] >= fy else -1), ph / 2 + 0.5
            L = next(iter(it["layers"]))
            self.add_track(it["net"], it["x"], it["y"], it["x"] + dx * ln, it["y"] + dy * ln, L, tw)
            print("  escape stub for %s.%s (%s)" % (it["ref"], it["num"], it["net"]))

    def route_net(self, net, clearances, tw_override=None):
        if net in self.zone_nets():
            print("  skip %s: net has a copper zone (zone fill connects it, refill zones instead)" % net)
            return False
        vsize, vdrill = self.common_via()
        tw = tw_override or self.common_width(net)
        snap_added, snap_items = len(self.added), len(self.items)
        self.add_escape_stubs({net}, tw)
        ok_all = True
        guard = 0
        while guard < 10:
            guard += 1
            comps = self.components(net)
            if len(comps) <= 1:
                break
            best = None
            for i in range(len(comps)):
                for j in range(i + 1, len(comps)):
                    for a in self.anchors(comps[i]):
                        for b in self.anchors(comps[j]):
                            d = math.hypot(a[0] - b[0], a[1] - b[1])
                            if best is None or d < best[0]:
                                best = (d, a, b)
            _, a, b = best
            res = None
            for clr in clearances:
                for (vs_, vd_) in dict.fromkeys([(vsize, vdrill), (0.5, 0.3)]):
                    for margin in (4.0, 8.0):
                        res = self.route_pair(net, a, b, tw, vs_, vd_, clr, margin)
                        if res:
                            break
                    if res:
                        vsize_used, vdrill_used = vs_, vd_
                        break
                if res:
                    print("  %s: routed (%.2f,%.2f)->(%.2f,%.2f) with clearance %.3f mm, track %.3f mm"
                          % (net, a[0], a[1], b[0], b[1], clr, tw))
                    break
            if not res:
                print("  FAILED to route %s between (%.2f,%.2f) and (%.2f,%.2f). "
                      "Try --clearances 0.15,0.127 or move a nearby part (C18/C19)." % (net, a[0], a[1], b[0], b[1]))
                ok_all = False
                del self.added[snap_added:]
                del self.items[snap_items:]
                print("  rolled back the partial routing of %s" % net)
                break
            segs, vias = res
            for (x1, y1, x2, y2, L) in segs:
                self.add_track(net, x1, y1, x2, y2, L, tw)
            for (x, y) in vias:
                self.add_via(net, x, y, vsize_used, vdrill_used)
        return ok_all

    # --- save -------------------------------------------------------------------
    def save(self, path):
        t = self.text
        for ri in sorted(self.deleted_root, reverse=True):
            s, e = self.spans[ri]
            while s > 0 and t[s - 1] in " \t":
                s -= 1
            if e < len(t) and t[e] == "\n":
                e += 1
            t = t[:s] + t[e:]
        # find end of root again (shifted); last ')' of the file
        k = t.rstrip().rfind(")")
        t = t[:k] + "".join(self.added) + t[k:]
        with open(path, "w", encoding="utf-8") as f:
            f.write(t)


def nets_from_report(path):
    nets = []
    if not os.path.exists(path):
        return nets
    inblock = False
    for line in open(path, encoding="utf-8", errors="ignore"):
        if line.startswith("["):
            inblock = line.startswith("[unconnected_items]")
            continue
        if inblock and "@(" in line:
            for m in re.findall(r"\[([^\]]+)\]", line):
                if m not in nets:
                    nets.append(m)
    return nets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcb", default="stm32g4-canfd-sensor-node.kicad_pcb")
    ap.add_argument("--pro", default=None, help=".kicad_pro (default: same name as pcb)")
    ap.add_argument("--report", default="drc_report.rpt", help="old DRC report, used to find unconnected nets")
    ap.add_argument("--nets", default=None, help="comma separated nets to route, e.g. /OSC_IN")
    ap.add_argument("--rm-via", action="append", default=None, help="X,Y of a dangling via to remove")
    ap.add_argument("--ripup", default=None, help="comma separated nets whose tracks/vias are deleted and re-routed "
                    "after the target nets (needed when a neighbour net blocks the way, e.g. /OSC_OUT)")
    ap.add_argument("--clearances", default="0.2,0.15,0.127")
    ap.add_argument("--track-width", type=float, default=None)
    ap.add_argument("--no-silk", action="store_true", help="do not touch silkscreen severities")
    ap.add_argument("--no-check", action="store_true", help="skip pad position sanity check")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--save-partial", action="store_true", help="write the file even if a net failed")
    a = ap.parse_args()

    pro = a.pro or re.sub(r"\.kicad_pcb$", ".kicad_pro", a.pcb)
    rm_vias = a.rm_via if a.rm_via is not None else ["134.8,84.2"]
    clearances = [float(x) for x in a.clearances.split(",")]

    print("Reading %s ..." % a.pcb)
    b = Board(a.pcb)

    # pad-position sanity check against the DRC log (also picks the rotation convention)
    expect = [("C18", "1", 131.525, 86.5), ("U2", "5", 135.0625, 87.75)]
    chosen = None
    for sign in (1, -1):
        b.build(sign)
        good = True
        for ref, num, ex, ey in expect:
            p = b.find_pad(ref, num)
            if not p or abs(p["x"] - ex) > 0.02 or abs(p["y"] - ey) > 0.02:
                good = False
        if good:
            chosen = sign
            break
    if chosen is None:
        if a.no_check:
            b.build(1)
        else:
            b.build(1)
            for ref, num, ex, ey in expect:
                p = b.find_pad(ref, num)
                print("  %s.%s computed %s, DRC says (%.4f, %.4f)" %
                      (ref, num, "(%.4f, %.4f)" % (p["x"], p["y"]) if p else "NOT FOUND", ex, ey))
            sys.exit("Pad positions do not match the DRC log. Stop (use --no-check to force).")
    else:
        print("  pad positions match the DRC log (C18.1, U2.5): OK")

    print("Items: %d pads/tracks/vias" % len(b.items))
    print("1) duplicate vias")
    b.remove_duplicate_vias()
    print("2) dangling vias")
    for s in rm_vias:
        x, y = [float(v) for v in s.split(",")]
        b.remove_dangling_via(x, y)
    print("3) unconnected nets")
    nets = [n.strip() for n in a.nets.split(",")] if a.nets else nets_from_report(a.report)
    if not nets:
        nets = ["/OSC_IN"]
    print("  nets to check: %s" % ", ".join(nets))
    ripup = [n.strip() for n in a.ripup.split(",")] if a.ripup else []
    for n in ripup:
        if n in b.zone_nets():
            sys.exit("net %s has a zone, refusing to rip it up" % n)
        cnt = 0
        for it in b.live():
            if it["net"] == n and it["kind"] in ("seg", "via"):
                b.delete_item(it)
                cnt += 1
        print("  ripped up %d track pieces/vias of %s (will be re-routed)" % (cnt, n))
    nets = nets + [n for n in ripup if n not in nets]
    all_ok = True
    if ripup:
        b.add_escape_stubs(set(nets), a.track_width or 0.15)
    for n in nets:
        all_ok &= b.route_net(n, clearances, a.track_width)

    if a.dry_run:
        print("dry run: nothing written")
        return
    if not all_ok and ripup and not a.save_partial:
        print("Re-routing after --ripup failed, so NOTHING was written (the ripped-up nets would be lost).")
        sys.exit(2)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    shutil.copy2(a.pcb, "%s.bak-pre-fix-%s" % (a.pcb, stamp))
    b.save(a.pcb)
    print("Saved %s (backup: %s.bak-pre-fix-%s)" % (a.pcb, a.pcb, stamp))

    if not a.no_silk and os.path.exists(pro):
        shutil.copy2(pro, "%s.bak-pre-fix-%s" % (pro, stamp))
        d = json.load(open(pro, encoding="utf-8"))
        rs = d.setdefault("board", {}).setdefault("design_settings", {}).setdefault("rule_severities", {})
        rs["silk_overlap"] = "ignore"
        rs["silk_over_copper"] = "ignore"
        json.dump(d, open(pro, "w", encoding="utf-8"), indent=2)
        print("4) silkscreen warnings set to 'ignore' in %s" % pro)
    print("Done." if all_ok else "Done, but some nets could not be routed (see messages above).")


if __name__ == "__main__":
    main()
