import numpy as np, trimesh

s = trimesh.load('mecanum.dae', force='scene')
print("scene units:", s.units)
print("num geometries:", len(s.geometry))

rows = []
for name, g in s.geometry.items():
    c = g.centroid
    ext = g.extents
    rows.append((name, len(g.faces), len(g.vertices), c, ext))

# sort by face count desc
rows.sort(key=lambda r: -r[1])
for name, nf, nv, c, ext in rows:
    print(f"{name[:40]:40s} f={nf:6d} v={nv:6d} c=[{c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}] ext=[{ext[0]:.4f},{ext[1]:.4f},{ext[2]:.4f}]")

# find the hub geometry (the big flat one) and inspect its connected components
print("\n--- biggest geometry connected components ---")
big = max(s.geometry.values(), key=lambda g: len(g.faces))
comps = big.split(only_watertight=False)
print("hub geom comps:", len(comps))
for i, comp in enumerate(sorted(comps, key=lambda c: -len(c.faces))[:20]):
    c = comp.centroid; ext = comp.extents
    print(f"  comp{i:2d} f={len(comp.faces):6d} c=[{c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}] ext=[{ext[0]:.4f},{ext[1]:.4f},{ext[2]:.4f}]")
