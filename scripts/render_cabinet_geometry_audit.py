"""Offline, orthographic CAD audit views; no simulator state is changed."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from sim_bridge.cabinet_geometry import triangles

fig=plt.figure(figsize=(14,9))
for i,(part,elev) in enumerate([('shell',65),('shell',-65),('mounting_panel',65),('rail_h1',65)],1):
    ax=fig.add_subplot(2,2,i,projection='3d')
    mesh=triangles(part)
    ax.add_collection3d(Poly3DCollection(mesh,facecolor='#8fa9be',edgecolor='#324659',linewidth=.08))
    lo=mesh.min(axis=(0,1));hi=mesh.max(axis=(0,1));center=(lo+hi)/2
    extent=max(hi-lo)/2
    ax.set_xlim(center[0]-extent,center[0]+extent)
    ax.set_ylim(center[1]-extent,center[1]+extent)
    ax.set_zlim(center[2]-extent,center[2]+extent)
    ax.set_box_aspect((1,1,1));ax.set_proj_type('ortho')
    ax.view_init(elev=elev,azim=-60)
    ax.set_title(f'{part}: viewed from {"+Z" if elev>0 else "-Z"}')
    ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Z (m)')
out=Path('data/audits/open_top_no_door_v2/cad_surfaces.png')
fig.tight_layout();fig.savefig(out,dpi=150)
print(out)
