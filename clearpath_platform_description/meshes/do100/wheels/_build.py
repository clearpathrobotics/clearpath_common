import numpy as np, trimesh

scene = trimesh.load('mecanum.dae', force='scene')
geoms = list(scene.geometry.values())

# ---- E-frame: hub axle -> +Y, centred at hub centroid ----
hub_geo = max(geoms, key=lambda g: len(g.faces))   # _053 (7194f)
c0 = hub_geo.centroid
Vc = hub_geo.vertices.view(np.ndarray) - c0
_, _, Vt = np.linalg.svd(Vc, full_matrices=False)
axle = Vt[2].copy()
if axle[0] < 0: axle = -axle

def rmat_a_to_b(a, b):
    a = a/np.linalg.norm(a); b = b/np.linalg.norm(b)
    v = np.cross(a, b); s = np.linalg.norm(v); c = np.dot(a, b)
    if s < 1e-9: return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3) + vx + vx@vx*((1-c)/(s*s))

R = rmat_a_to_b(axle, np.array([0.0,1.0,0.0]))

def to_E(verts): return (R @ (verts - c0).T).T
def roty(t):
    ct, st = np.cos(t), np.sin(t)
    return np.array([[ct,0,st],[0,1,0],[-st,0,ct]])

# ---- barrels (9 biggest after hub) vs small parts (caps / nuts / nails) ----
barrels = sorted([g for g in geoms if g is not hub_geo], key=lambda g: -len(g.faces))[:9]
small   = [g for g in geoms if g is not hub_geo and g not in barrels]

def axis_of(g):
    Ve = to_E(g.vertices.view(np.ndarray)); m = Ve.mean(0)
    _, _, vt = np.linalg.svd(Ve - m, full_matrices=False)
    return m, vt[0]
bar_info = [axis_of(b) for b in barrels]

def perp(p, c, d):
    w = p - c; return np.linalg.norm(w - np.dot(w, d)*d)

# classify each small part as NUT (compact) or NAIL (long & thin)
def kind(g):
    emin, emid, emax = np.sort(g.extents)
    if emax / max(emid, 1e-6) >= 2.5 and emid < 0.004:
        return 'nail'
    return 'nut'

nuts_all  = [g for g in small if kind(g) == 'nut']
nails_all = [g for g in small if kind(g) == 'nail']
print(f"small parts: {len(small)}  -> nuts={len(nuts_all)} nails={len(nails_all)}")

# ---- HUB = plate + all NUT caps (nails excluded), in E-frame ----
hub_parts = []
m = hub_geo.copy(); m.vertices = to_E(m.vertices.view(np.ndarray)); hub_parts.append(m)
for g in nuts_all:
    mm = g.copy(); mm.vertices = to_E(g.vertices.view(np.ndarray)); hub_parts.append(mm)
hub = trimesh.util.concatenate(hub_parts); hub.merge_vertices()
print(f"HUB  plate+nuts faces={len(hub.faces)} ext={np.round(hub.extents,4)}")

# ---- ROLLER = bare barrel only (canonical = barrel nearest ring-angle 0) ----
def angle_of(c): return np.arctan2(-c[2], c[0])
cano = min(range(9), key=lambda k: abs(angle_of(bar_info[k][0])))
cc, _ = bar_info[cano]; phi = angle_of(cc); r = np.hypot(cc[0], cc[2])
print(f"canonical barrel #{cano} angle={np.degrees(phi):.1f} r={r:.4f}")

roller = barrels[cano].copy()
Ve = to_E(roller.vertices.view(np.ndarray))
roller.vertices = (roty(-phi) @ Ve.T).T - np.array([r, 0.0, 0.0])
roller.merge_vertices()
print(f"ROLLER barrel  faces={len(roller.faces)} ext={np.round(roller.extents,4)}")

hub.export('_hub_exact.stl')
roller.export('_roller_exact.stl')

# ---- preview ----
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
def panel(ax, mesh, title, color='0.6', elev=18, azim=-60):
    ax.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], facecolor=color, edgecolor='none'))
    b = mesh.bounds; ctr = b.mean(0); rad = (b[1]-b[0]).max()/2
    for s, lo in zip('xyz', ctr-rad): getattr(ax,f'set_{s}lim')(lo, lo+2*rad)
    ax.set_title(title, fontsize=9); ax.set_box_aspect((1,1,1)); ax.view_init(elev, azim); ax.set_axis_off()
fig = plt.figure(figsize=(14,5))
panel(fig.add_subplot(131, projection='3d'), hub, f'HUB plate+nuts ({len(hub.faces)}f)')
panel(fig.add_subplot(132, projection='3d'), hub, 'HUB face-on', '0.6', 90, -90)
panel(fig.add_subplot(133, projection='3d'), roller, f'ROLLER barrel only ({len(roller.faces)}f)')
plt.tight_layout(); plt.savefig('_preview.png', dpi=95)
print('wrote _preview.png')
