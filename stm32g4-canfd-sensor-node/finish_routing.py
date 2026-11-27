#!/usr/bin/env python3
"""Finish routing (stdlib only, edits the .kicad_pcb text; KiCad must be closed).

For every unconnected pair reported by DRC:
  1. pad->pad pattern: 0.15mm stub (2-3mm) -> via -> 0.40mm trace on another layer -> via -> 0.15mm stub
  2. fallback: multi-layer grid A* (F.Cu/In2.Cu/B.Cu), from/to pads, dangling track ends or vias;
     0.15mm near the ends, widened to 0.4 (0.3/0.25/0.2) in the middle when space allows.
Final DRC decides; if DRC errors increase, the board is restored from backup.

Usage:  python3 finish_routing.py stm32g4-canfd-sensor-node.kicad_pcb
Env:    CLR=0.2   own clearance estimate (mm)
"""
import sys, os, re, json, glob, math, shutil, subprocess, uuid, heapq
from collections import Counter

BOARD = sys.argv[1]
CLR = float(os.environ.get('CLR', '0.2'))
MARGIN, EDGE = 0.03, 0.35
W_STUB, W_END, W_GRID = 0.15, 0.15, 0.15
W_MID = [0.40, 0.30, 0.25, 0.20]
LAY = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
ROUTE = ['F.Cu', 'In2.Cu', 'B.Cu']
G, VIA_COST, MAX_EXPAND = 0.05, 4.0, 1500000
CLI = (sorted(glob.glob('/tmp/.mount_kicad*/usr/bin/kicad-cli')) or ['kicad-cli'])[-1]

# ------------------------------------------------------------ s-expression
class Q(str): pass
TOK = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+')
def parse(text):
    st = [[]]
    for t in TOK.findall(text):
        if t == '(': st.append([])
        elif t == ')': x = st.pop(); st[-1].append(x)
        elif t[0] == '"': st[-1].append(Q(t[1:-1]))
        else: st[-1].append(t)
    return st[0][0]
def kids(n, name): return [c for c in n if isinstance(c, list) and c and c[0] == name]
def kid(n, name):
    k = kids(n, name); return k[0] if k else None

# ------------------------------------------------------------ geometry
def pt_seg(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]; L = dx*dx+dy*dy
    t = 0 if L == 0 else max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/L))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)
def ccw(a, b, c): return (c[1]-a[1])*(b[0]-a[0])-(b[1]-a[1])*(c[0]-a[0])
def seg_seg(a, b, c, d):
    if ccw(a, c, d)*ccw(b, c, d) < 0 and ccw(a, b, c)*ccw(a, b, d) < 0: return 0
    return min(pt_seg(a, c, d), pt_seg(b, c, d), pt_seg(c, a, b), pt_seg(d, a, b))
def inside(p, r): return r[0] <= p[0] <= r[2] and r[1] <= p[1] <= r[3]
def seg_rect(a, b, r):
    if inside(a, r) or inside(b, r): return 0
    c = [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]
    return min(seg_seg(a, b, c[i], c[(i+1) % 4]) for i in range(4))
def rot(x, y, a):
    r = math.radians(a); c, s = math.cos(r), math.sin(r)
    return x*c+y*s, -x*s+y*c
def dist(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
def snap(v, g=G): return round(v/g)*g
def fmt(v): return ('%.4f' % v).rstrip('0').rstrip('.')

def run_drc(path, out, save=False):
    cmd = [CLI, 'pcb', 'drc', '--refill-zones'] + (['--save-board'] if save else []) + \
          ['--severity-all', '--format', 'json', '--output', out, path]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    r = json.load(open(out))
    return r, sum(v.get('severity') == 'error' for v in r.get('violations', []))

# ------------------------------------------------------------ board model
class Model:
    def __init__(self, text):
        self.root = root = parse(text)
        self.id2name, self.name2id = {}, {}
        for n in kids(root, 'net'):
            if len(n) >= 3: self.id2name[str(n[1])] = str(n[2]); self.name2id[str(n[2])] = str(n[1])
        self.obs = {l: [] for l in LAY}
        self.pads, self.segs, self.layer_use = [], [], Counter()
        xs, ys = [], []
        for e in root:
            if isinstance(e, list) and e and str(e[0]).startswith('gr_'):
                lay = kid(e, 'layer')
                if lay and str(lay[1]) == 'Edge.Cuts':
                    for k in ('start', 'end', 'mid', 'center'):
                        p = kid(e, k)
                        if p: xs.append(float(p[1])); ys.append(float(p[2]))
                    pts = kid(e, 'pts')
                    for p in (kids(pts, 'xy') if pts else []): xs.append(float(p[1])); ys.append(float(p[2]))
        self.xmin, self.xmax, self.ymin, self.ymax = (min(xs), max(xs), min(ys), max(ys)) if xs else (-1e9, 1e9, -1e9, 1e9)
        for fp in kids(root, 'footprint'):
            at = kid(fp, 'at'); fx, fy = float(at[1]), float(at[2]); fa = float(at[3]) if len(at) > 3 else 0
            for pd in kids(fp, 'pad'):
                pa_ = kid(pd, 'at'); px, py = float(pa_[1]), float(pa_[2]); pa = float(pa_[3]) if len(pa_) > 3 else 0
                rx, ry = rot(px, py, fa); X, Y = fx+rx, fy+ry
                sz = kid(pd, 'size'); w, h = float(sz[1]), float(sz[2])
                if round(pa) % 180 == 90: w, h = h, w
                elif round(pa) % 90 != 0: w = h = max(w, h)
                ls = [str(l) for l in (kid(pd, 'layers') or [])[1:]]
                ll = LAY if '*.Cu' in ls else [l for l in ls if l in LAY]
                net = self.norm(kid(pd, 'net'))
                rect = (X-w/2, Y-h/2, X+w/2, Y+h/2)
                for l in ll: self.obs[l].append(('r', rect, net, rect))
                self.pads.append(dict(pos=(X, Y), net=net, layers=ll, rect=rect, size=(w, h)))
        for s in kids(root, 'segment'):
            a, b = kid(s, 'start'), kid(s, 'end')
            self.add_seg((float(a[1]), float(a[2])), (float(b[1]), float(b[2])), float(kid(s, 'width')[1]),
                         str(kid(s, 'layer')[1]), self.norm(kid(s, 'net')))
        for s in kids(root, 'arc'):
            P = [(float(kid(s, k)[1]), float(kid(s, k)[2])) for k in ('start', 'mid', 'end')]
            w, l, n = float(kid(s, 'width')[1]), str(kid(s, 'layer')[1]), self.norm(kid(s, 'net'))
            self.add_seg(P[0], P[1], w, l, n); self.add_seg(P[1], P[2], w, l, n)
        vs = Counter()
        for v in kids(root, 'via'):
            at = kid(v, 'at'); sz, dr = float(kid(v, 'size')[1]), float(kid(v, 'drill')[1])
            vs[(sz, dr)] += 1; self.add_via((float(at[1]), float(at[2])), sz, self.norm(kid(v, 'net')))
        self.vsize, self.vdrill = vs.most_common(1)[0][0] if vs else (0.6, 0.3)
        sg = kids(root, 'segment')
        self.net_id_style = bool(sg and len(kid(sg[0], 'net')) == 2 and not isinstance(kid(sg[0], 'net')[1], Q))
    def norm(self, n):
        if not n or len(n) < 2: return None
        if len(n) >= 3: return str(n[2])
        v = str(n[1])
        return v if isinstance(n[1], Q) else self.id2name.get(v, v)
    def add_seg(self, a, b, w, layer, net):
        self.segs.append((a, b, w, layer, net))
        if layer not in self.obs: return
        r = w/2; bb = (min(a[0], b[0])-r, min(a[1], b[1])-r, max(a[0], b[0])+r, max(a[1], b[1])+r)
        self.obs[layer].append(('s', (a, b, r), net, bb)); self.layer_use[layer] += 1
    def add_via(self, c, size, net):
        r = size/2; bb = (c[0]-r, c[1]-r, c[0]+r, c[1]+r)
        for l in LAY: self.obs[l].append(('c', (c, r), net, bb))
    def blocked(self, a, b, rn, layers, net):
        x0, y0 = min(a[0], b[0])-rn-CLR-1, min(a[1], b[1])-rn-CLR-1
        x1, y1 = max(a[0], b[0])+rn+CLR+1, max(a[1], b[1])+rn+CLR+1
        lim = CLR+MARGIN
        for p in (a, b):
            if not (self.xmin+EDGE+rn <= p[0] <= self.xmax-EDGE-rn and self.ymin+EDGE+rn <= p[1] <= self.ymax-EDGE-rn):
                return True
        for L in layers:
            for kind, g, n, bb in self.obs[L]:
                if n is not None and n == net: continue
                if bb[2] < x0 or bb[0] > x1 or bb[3] < y0 or bb[1] > y1: continue
                if kind == 's': d = seg_seg(a, b, g[0], g[1])-rn-g[2]
                elif kind == 'c': d = pt_seg(g[0], a, b)-rn-g[1]
                else: d = seg_rect(a, b, g)-rn
                if d < lim: return True
        return False
    def nettxt(self, net):
        if self.net_id_style and net in self.name2id: return '(net %s)' % self.name2id[net]
        return '(net "%s")' % net
    def segtxt(self, a, b, w, layer, net):
        return '(segment (start %s %s) (end %s %s) (width %s) (layer "%s") %s (uuid "%s"))' % (
            fmt(a[0]), fmt(a[1]), fmt(b[0]), fmt(b[1]), fmt(w), layer, self.nettxt(net), uuid.uuid4())
    def viatxt(self, c, net):
        return '(via (at %s %s) (size %s) (drill %s) (layers "F.Cu" "B.Cu") %s (uuid "%s"))' % (
            fmt(c[0]), fmt(c[1]), fmt(self.vsize), fmt(self.vdrill), self.nettxt(net), uuid.uuid4())

def pad_layer(p): return 'F.Cu' if 'F.Cu' in p['layers'] else 'B.Cu'

GEOM = []   # geometry points of the route being built (used for rip-up proximity)

def commit(M, items, segs, vias, net):
    for a, b, w, l in segs:
        items.append(M.segtxt(a, b, w, l, net)); M.add_seg(a, b, w, l, net); GEOM.extend([a, b])
    for c in vias:
        items.append(M.viatxt(c, net)); M.add_via(c, M.vsize, net); GEOM.append(c)

# ------------------------------------------------------------ stub candidates
def stub_candidates(M, pad, target, layer, net, lens):
    c = pad['pos']; base = math.atan2(target[1]-c[1], target[0]-c[0]); out, seen = [], set()
    angs = [base+k*math.pi/4 for k in range(8)] + [k*math.pi/4 for k in range(8)]
    for th in angs:
        for L in lens:
            p = (snap(c[0]+L*math.cos(th)), snap(c[1]+L*math.sin(th)))
            if p in seen: continue
            seen.add(p)
            if not M.blocked(c, p, W_STUB/2, [layer], net): out.append((L, p))
    return out

# ------------------------------------------------------------ 1) pattern router (pad -> pad)
def route_pattern(M, A, B, net, items):
    la, lb = pad_layer(A), pad_layer(B)
    vr = M.vsize/2
    S = [(L, p) for L, p in stub_candidates(M, A, B['pos'], la, net, [2.0, 2.5, 3.0]) if not M.blocked(p, p, vr, LAY, net)]
    E = [(L, p) for L, p in stub_candidates(M, B, A['pos'], lb, net, [2.0, 2.5, 3.0]) if not M.blocked(p, p, vr, LAY, net)]
    combos = sorted(((dist(s[1], e[1]), s[1], e[1]) for s in S for e in E
                     if dist(s[1], e[1]) > M.vsize+CLR+0.1), key=lambda x: x[0])
    for mid in sorted(['B.Cu', 'In2.Cu'], key=lambda l: M.layer_use[l]):
        if mid in (la, lb): continue
        for _, P, Q_ in combos:
            for w in W_MID:
                for path in ([P, Q_], [P, (Q_[0], P[1]), Q_], [P, (P[0], Q_[1]), Q_]):
                    sg = [(path[i], path[i+1]) for i in range(len(path)-1) if dist(path[i], path[i+1]) > 1e-6]
                    if any(M.blocked(a, b, w/2, [mid], net) for a, b in sg): continue
                    segs = [(A['pos'], P, W_STUB, la)] + [(a, b, w, mid) for a, b in sg] + [(Q_, B['pos'], W_END, lb)]
                    commit(M, items, segs, [P, Q_], net)
                    return 'pattern ' + mid + ' w=%.2f' % w
    return None

# ------------------------------------------------------------ 2) grid A*
def fill(arr, x0, y0, Wd, Hd, kind, g, R):
    if kind == 's': a, b, Re = g[0], g[1], R+g[2]
    elif kind == 'c': a = b = g[0]; Re = R+g[1]
    else: a = b = None; Re = R
    if kind == 'r': bx0, by0, bx1, by1 = g[0]-R, g[1]-R, g[2]+R, g[3]+R
    else: bx0, by0, bx1, by1 = min(a[0], b[0])-Re, min(a[1], b[1])-Re, max(a[0], b[0])+Re, max(a[1], b[1])+Re
    ix0, ix1 = max(0, int((bx0-x0)/G)), min(Wd-1, int((bx1-x0)/G)+1)
    iy0, iy1 = max(0, int((by0-y0)/G)), min(Hd-1, int((by1-y0)/G)+1)
    for iy in range(iy0, iy1+1):
        py = y0+iy*G
        for ix in range(ix0, ix1+1):
            px = x0+ix*G
            if kind == 'r':
                d = math.hypot(max(g[0]-px, 0, px-g[2]), max(g[1]-py, 0, py-g[3]))
            else: d = pt_seg((px, py), a, b)
            if d <= Re: arr[iy*Wd+ix] = 1

def build_grid(M, net, reg, gm):
    x0, y0, x1, y1 = reg
    Wd, Hd = int(round((x1-x0)/G))+1, int(round((y1-y0)/G))+1
    tr = {l: bytearray(Wd*Hd) for l in ROUTE}; via = bytearray(Wd*Hd)
    Rt, Rv = W_GRID/2+CLR+gm, M.vsize/2+CLR+gm; seen = set()
    for L in LAY:
        for kind, g, n, bb in M.obs[L]:
            if n is not None and n == net: continue
            if bb[2] < x0-Rv-1 or bb[0] > x1+Rv+1 or bb[3] < y0-Rv-1 or bb[1] > y1+Rv+1: continue
            if L in tr: fill(tr[L], x0, y0, Wd, Hd, kind, g, Rt)
            if id(g) not in seen: seen.add(id(g)); fill(via, x0, y0, Wd, Hd, kind, g, Rv)
    return dict(x0=x0, y0=y0, W=Wd, H=Hd, tr=tr, via=via)

def endpoint_nodes(M, ep, net, lens, grid):
    """list of dict(ix,iy,li,cost,segs) ; segs = prefix segments [(a,b,w,layer)] from the endpoint outward"""
    x0, y0, Wd, Hd = grid['x0'], grid['y0'], grid['W'], grid['H']
    def cell(p): return int(round((p[0]-x0)/G)), int(round((p[1]-y0)/G))
    def ok(ix, iy): return 0 <= ix < Wd and 0 <= iy < Hd
    out = []
    if ep['kind'] == 'pad':
        pad = ep['pad']; w, h = pad['size']; lays = [l for l in pad['layers'] if l in ROUTE]
        if min(w, h) >= 1.0:
            r = pad['rect']; x = r[0]+0.3
            while x <= r[2]-0.3:
                y = r[1]+0.3
                while y <= r[3]-0.3:
                    ix, iy = cell((x, y))
                    if ok(ix, iy):
                        for l in lays: out.append(dict(ix=ix, iy=iy, li=ROUTE.index(l), cost=0, segs=[]))
                    y += 0.25
                x += 0.25
        else:
            la = pad_layer(pad)
            if la in ROUTE:
                # also let A* leave the pad centre freely (no fixed stub)
                cix, ciy = cell(pad['pos']); cgp = (x0+cix*G, y0+ciy*G)
                if ok(cix, ciy):
                    cs = [(pad['pos'], cgp, W_GRID, la)] if dist(pad['pos'], cgp) > 1e-6 else []
                    out.append(dict(ix=cix, iy=ciy, li=ROUTE.index(la), cost=0.0, segs=cs))
                for L, p in stub_candidates(M, pad, ep['toward'], la, net, lens):
                    ix, iy = cell(p)
                    if ok(ix, iy):
                        out.append(dict(ix=ix, iy=iy, li=ROUTE.index(la), cost=L, segs=[(pad['pos'], p, W_STUB, la)]))
    else:
        p = ep['pt']; ix, iy = cell(p); gp = (x0+ix*G, y0+iy*G)
        if ok(ix, iy):
            lays = ROUTE if ep['kind'] == 'via' else ([ep['layer']] if ep['layer'] in ROUTE else [])
            for l in lays:
                segs = [(p, gp, W_GRID, l)] if dist(p, gp) > 1e-6 else []
                out.append(dict(ix=ix, iy=iy, li=ROUTE.index(l), cost=0, segs=segs))
    return out

def astar(grid, seeds, goals):
    Wd, Hd = grid['W'], grid['H']; HW = Wd*Hd; trl = [grid['tr'][l] for l in ROUTE]; via = grid['via']
    for n in seeds + goals: trl[n['li']][n['iy']*Wd+n['ix']] = 0
    gidx = {(n['li']*Hd+n['iy'])*Wd+n['ix']: n for n in goals}
    sidx = {(n['li']*Hd+n['iy'])*Wd+n['ix']: n for n in seeds}
    gc = (sum(n['ix'] for n in goals)/len(goals), sum(n['iy'] for n in goals)/len(goals))
    gsp = max(math.hypot(n['ix']-gc[0], n['iy']-gc[1]) for n in goals)
    def h(ix, iy): return max(0.0, (math.hypot(ix-gc[0], iy-gc[1])-gsp)*G)
    INF = 1e18; N = len(ROUTE)*HW
    gs = [INF]*N; par = [-1]*N; heap = []
    for i, n in sidx.items():
        c = n['cost']
        if c < gs[i]: gs[i] = c; heapq.heappush(heap, (c+h(n['ix'], n['iy']), c, i))
    D = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    exp = 0
    while heap:
        f, g, i = heapq.heappop(heap)
        if g > gs[i]: continue
        if i in gidx:
            path = []
            while i != -1: path.append(i); i = par[i]
            return path[::-1], sidx[path[-1]], gidx[path[0]]
        exp += 1
        if exp > MAX_EXPAND: return None
        li, rem = divmod(i, HW); iy, ix = divmod(rem, Wd); arr = trl[li]
        for dx, dy, c in D:
            nx, ny = ix+dx, iy+dy
            if 0 <= nx < Wd and 0 <= ny < Hd and not arr[ny*Wd+nx]:
                j = i+dy*Wd+dx; ng = g+c*G
                if ng < gs[j]: gs[j] = ng; par[j] = i; heapq.heappush(heap, (ng+h(nx, ny), ng, j))
        if not via[iy*Wd+ix]:
            for lj in range(len(ROUTE)):
                if lj != li and not trl[lj][iy*Wd+ix]:
                    j = lj*HW+iy*Wd+ix; ng = g+VIA_COST
                    if ng < gs[j]: gs[j] = ng; par[j] = i; heapq.heappush(heap, (ng+h(ix, iy), ng, j))
    return None

def path_to_geometry(grid, path):
    x0, y0, Wd, Hd = grid['x0'], grid['y0'], grid['W'], grid['H']; HW = Wd*Hd
    nodes = []
    for i in path:
        li, rem = divmod(i, HW); iy, ix = divmod(rem, Wd); nodes.append((ix, iy, li))
    segs, vias = [], []; start = nodes[0]; cur_dir = None
    def P(n): return (x0+n[0]*G, y0+n[1]*G)
    prev = nodes[0]
    for n in nodes[1:]:
        if n[2] != prev[2]:
            if cur_dir is not None: segs.append([P(start), P(prev), ROUTE[prev[2]]]); cur_dir = None
            vias.append(P(prev)); start = n
        else:
            d = (n[0]-prev[0], n[1]-prev[1])
            if cur_dir is None: cur_dir = d
            elif d != cur_dir:
                segs.append([P(start), P(prev), ROUTE[prev[2]]]); start = prev; cur_dir = d
        prev = n
    if cur_dir is not None: segs.append([P(start), P(prev), ROUTE[prev[2]]])
    return segs, vias

def route_grid(M, epA, epB, net, items):
    global MARGIN, DIAG
    DIAG = []
    FINE = [round(0.3+0.1*i, 1) for i in range(28)]
    pts = [epA['pt'], epB['pt']]
    reg = (max(snap(min(p[0] for p in pts)-5), M.xmin+EDGE+0.2), max(snap(min(p[1] for p in pts)-5), M.ymin+EDGE+0.2),
           min(snap(max(p[0] for p in pts)+5), M.xmax-EDGE-0.2), min(snap(max(p[1] for p in pts)+5), M.ymax-EDGE-0.2))
    reg = tuple(snap(v) for v in reg)
    for gm in (0.03, 0.08, 0.0):
        grid = build_grid(M, net, reg, gm)
        for lens in ([2.0, 2.5, 3.0], [0.8, 1.2, 1.6], FINE):
            seeds, goals = endpoint_nodes(M, epA, net, lens, grid), endpoint_nodes(M, epB, net, lens, grid)
            if not seeds or not goals:
                DIAG.append('gm=%.2f stubs=%d: start_seeds=%d goal_seeds=%d' % (gm, len(lens), len(seeds), len(goals))); continue
            res = astar(grid, seeds, goals)
            if not res:
                DIAG.append('gm=%.2f stubs=%d: seeds=%d/%d but no path in region' % (gm, len(lens), len(seeds), len(goals))); continue
            path, sn, gn = res
            segs, vias = path_to_geometry(grid, path)
            # widen the middle (>=2.5mm away from both ends)
            total = sum(dist(a, b) for a, b, l in segs); run = 0.0; fin = []
            for a, b, l in segs:
                d = dist(a, b); w = W_GRID
                if d >= 0.8 and run >= 2.5 and run+d <= total-2.5:
                    for cand in W_MID:
                        if not M.blocked(a, b, cand/2, [l], net): w = cand; break
                fin.append((a, b, w, l)); run += d
            allsegs = list(sn['segs']) + fin + list(gn['segs'])
            if any(M.blocked(a, b, w/2, [l], net) for a, b, w, l in allsegs):
                DIAG.append('gm=%.2f: path found but a segment violates clearance' % gm); continue
            if any(M.blocked(v, v, M.vsize/2, LAY, net) for v in vias):
                DIAG.append('gm=%.2f: path found but a via violates clearance' % gm); continue
            commit(M, items, allsegs, vias, net)
            return 'grid vias=%d' % len(vias)
    return None

# ------------------------------------------------------------ endpoints from DRC items
def candidates(M, it, name):
    x, y = it['pos']['x'], it['pos']['y']; d = it.get('description', '')
    if d.startswith('Pad'):
        p = min(M.pads, key=lambda q: dist(q['pos'], (x, y)))
        if dist(p['pos'], (x, y)) < 0.5: return [dict(kind='pad', pad=p, pt=p['pos'])]
    if d.startswith('Via'): return [dict(kind='via', pt=(x, y))]
    if d.startswith('Track'):
        m = re.search(r'on ([\w.]+),', d); lay = m.group(1) if m else 'F.Cu'; out = []
        for a, b, w, l, n in M.segs:
            if l == lay and (dist(a, (x, y)) < 0.02 or dist(b, (x, y)) < 0.02):
                out += [dict(kind='trk', pt=a, layer=l), dict(kind='trk', pt=b, layer=l)]
        return out
    return []

def main():
    d = os.path.dirname(os.path.abspath(BOARD))
    if os.path.exists(os.path.join(d, '~' + os.path.basename(BOARD) + '.lck')):
        sys.exit('LOCK present: close the board in KiCad first.')
    bak = BOARD + '.bak-pre-finish'; shutil.copy(BOARD, bak)
    r0, e0 = run_drc(BOARD, '/tmp/drc-before.json')
    text = open(BOARD).read(); M = Model(text)
    global DIAG
    DIAG = []
    items, ok, fail = [], 0, 0
    for u in r0.get('unconnected_items', []):
        it = u.get('items', [])
        names = {n for x in it for n in re.findall(r'\[([^\]]+)\]', x.get('description', ''))}
        if len(it) != 2 or not names: continue
        name = sorted(names)[0]
        ca, cb = candidates(M, it[0], name), candidates(M, it[1], name)
        if not ca or not cb: print('SKIP (unknown endpoint)', name, it[0].get('description', '')[:40]); fail += 1; continue
        A, B = min(((a, b) for a in ca for b in cb), key=lambda ab: dist(ab[0]['pt'], ab[1]['pt']))
        A['toward'], B['toward'] = B['pt'], A['pt']
        res = None
        if A['kind'] == 'pad' and B['kind'] == 'pad' and min(A['pad']['size']) < 1 and min(B['pad']['size']) < 1:
            res = route_pattern(M, A['pad'], B['pad'], name, items)
        if not res: res = route_grid(M, A, B, name, items)
        print('OK  ' if res else 'FAIL', name, res or '', '(%.1f,%.1f)->(%.1f,%.1f)' % (A['pt'] + B['pt']))
        if not res:
            for line in dict.fromkeys(DIAG): print('      why:', line)
        ok += bool(res); fail += (not res)
    if items:
        open(BOARD, 'w').write(text.rstrip()[:-1].rstrip() + '\n  ' + '\n  '.join(items) + '\n)\n')
    r1, e1 = run_drc(BOARD, '/tmp/drc-after.json', save=True)
    print('routed=%d failed=%d unconnected %d->%d drc_errors %d->%d' % (
        ok, fail, len(r0.get('unconnected_items', [])), len(r1.get('unconnected_items', [])), e0, e1))
    if e1 > e0:
        shutil.copy(bak, BOARD)
        print('DRC errors increased -> restored backup (%s).' % bak)
        for v in r1.get('violations', []):
            if v.get('severity') == 'error': print('  ', v.get('type'), [i.get('description', '')[:50] for i in v.get('items', [])])

if __name__ == '__main__':
    main()
