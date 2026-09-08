"""Geometry queries in the canonical CAD frame (metres, cabinet front up)."""
from functools import lru_cache
from pathlib import Path
import numpy as np

PROCESSED = Path(__file__).resolve().parents[1] / 'models/cabinet/processed'

@lru_cache(maxsize=32)
def triangles(part_id: str) -> np.ndarray:
    data = (PROCESSED / f'{part_id}.stl').read_bytes()
    dtype = np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')])
    return np.frombuffer(data[84:],dtype=dtype)['v'].astype(float)

def axis_intersections(mesh: np.ndarray, point: tuple[float,float], axis: int=2) -> np.ndarray:
    """Intersect an axis-parallel line with the real triangle surfaces."""
    uv = [i for i in range(3) if i != axis]
    a,b,c = mesh[:,0],mesh[:,1],mesh[:,2]
    u=b-a;v=c-a
    p=np.asarray(point)-a[:,uv]
    det=u[:,uv[0]]*v[:,uv[1]]-u[:,uv[1]]*v[:,uv[0]]
    valid=np.abs(det)>1e-14
    safe=np.where(valid,det,1)
    s=(p[:,0]*v[:,uv[1]]-p[:,1]*v[:,uv[0]])/safe
    t=(u[:,uv[0]]*p[:,1]-u[:,uv[1]]*p[:,0])/safe
    keep=valid & (s>=-1e-8) & (t>=-1e-8) & (s+t<=1+1e-8)
    return np.unique(np.round((a[:,axis]+s*u[:,axis]+t*v[:,axis])[keep],8))

def top_surface_z(part_id: str, xy: tuple[float,float]) -> float:
    hits=axis_intersections(triangles(part_id),xy)
    if not len(hits):
        raise ValueError(f'{part_id}: no material under grasp point {xy}')
    return float(hits.max())


def flat_patch_height(part_id: str, xy: tuple[float,float], radius: float,
                      tolerance: float=0.0005) -> float:
    """Require material at the centre and around the entire cup footprint."""
    heights = [top_surface_z(part_id,xy)]
    for angle in np.linspace(0,2*np.pi,16,endpoint=False):
        heights.append(top_surface_z(part_id,(xy[0]+radius*np.cos(angle),xy[1]+radius*np.sin(angle))))
    if np.ptp(heights)>tolerance:
        raise ValueError(f'{part_id}: surface variation exceeds suction-pad tolerance')
    return heights[0]


def require_open_top_shell() -> None:
    """Reject an inverted cabinet even when its bounding dimensions match.

    Central access columns must not hit a roof in the upper half of the
    shell. This is a necessary orientation check, not full path validation.
    """
    mesh=triangles('shell')
    lo=mesh.min(axis=(0,1));hi=mesh.max(axis=(0,1))
    mid=(lo+hi)/2
    for fx in (-.25,0.,.25):
        xy=(mid[0]+fx*(hi[0]-lo[0]),mid[1])
        hits=axis_intersections(mesh,xy)
        if any(h>mid[2] for h in hits):
            raise RuntimeError(
                'Cabinet orientation audit failed: solid shell roof blocks '
                f'top access at CAD XY={xy}, Z={hits.tolist()}. '
                'Correct the opening direction and resolve mounting-panel/door '
                'identity before planning or replay; do not waive this collision.'
            )


@lru_cache(maxsize=16)
def vacuum_grasp_point(part_id: str, radius: float=.006) -> tuple[float,float,float]:
    """Find the closest flat, real surface patch to the part's XY centre."""
    mesh=triangles(part_id)
    lo=mesh.min(axis=(0,1));hi=mesh.max(axis=(0,1));mid=(lo+hi)/2
    offsets=sorted(((x,y) for x in np.arange(-.012,.0121,.002)
                    for y in np.arange(-.012,.0121,.002)),key=lambda p:p[0]**2+p[1]**2)
    for dx,dy in offsets:
        xy=(float(mid[0]+dx),float(mid[1]+dy))
        if any(xy[i]-radius<lo[i] or xy[i]+radius>hi[i] for i in (0,1)):
            continue
        try:
            z=flat_patch_height(part_id,xy,radius)
            return (*xy,z)
        except ValueError:
            continue
    raise ValueError(f'{part_id}: no verified flat patch for diameter {2*radius} m')
