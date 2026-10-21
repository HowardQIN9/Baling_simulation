"""Finite spear measurements in bale coordinates; poses use xyz + wxyz."""
import math
from itertools import product


def rotate(q, v):
    w, x, y, z = q
    norm = math.sqrt(sum(a*a for a in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid rotation quaternion")
    w, x, y, z = (a/norm for a in (w, x, y, z))
    return [
        (1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2],
        2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2],
        2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2],
    ]


def transform(pose, point):
    return [a+b for a, b in zip(pose[:3], rotate(pose[3:], point))]


def inverse_point(pose, point):
    w, x, y, z = pose[3:]
    return rotate([w, -x, -y, -z], [a-b for a, b in zip(point, pose[:3])])


def segment_overlap(origin, tip, half):
    """Actual finite centerline length inside the visual box, including tilt."""
    lo, hi = 0., 1.
    delta = [b-a for a, b in zip(origin, tip)]
    for p, v, extent in zip(origin, delta, half):
        if abs(v) < 1e-12:
            if abs(p) > extent:
                return 0.
        else:
            entry, exit = sorted(((-extent-p)/v, (extent-p)/v))
            lo, hi = max(lo, entry), min(hi, exit)
    return max(0., hi-lo)*math.sqrt(sum(v*v for v in delta))


def bale_attitude(config, pose):
    b = config['bale']
    corners = product(*[[-s/2, s/2] for s in (b['width'], b['length'], b['height'])])
    bottom = min(transform(pose, p)[2] for p in corners)
    up = rotate(pose[3:], [0., 0., 1.])
    return dict(bottom_height_m=bottom,
                tilt_deg=math.degrees(math.acos(max(-1., min(1., up[2])))))


def engagement(config, carrier, bale):
    """Report all shafts, without treating box overlap as contact or support.

    Signed depth is tip progress past the near bale X face. Channel clearance
    uses the finite shaft portion inside the bale, including shaft radius.
    """
    a, b, ch = (config[key] for key in ("attachment", "bale", "channel"))
    spacing = (a["frame_width"]-2*a["side_margin"])/(a["count"]-1)
    radius = a["diameter"]/2
    half = [b["width"]/2, b["length"]/2, b["height"]/2]
    channel_z = ch["height_from_bottom"]-half[2]
    shafts = []
    for i in range(a["count"]):
        y = (i-(a["count"]-1)/2)*spacing
        root = inverse_point(bale, transform(carrier, [a["root_x"], y, a["root_z"]]))
        tip = inverse_point(bale, transform(carrier, [a["root_x"]+a["spear_length"], y, a["root_z"]]))
        dx = tip[0]-root[0]
        clear = False
        material_clear = False
        if dx > 1e-9:
            low = max(0., (-half[0]-root[0])/dx)
            high = min(1., (half[0]-root[0])/dx)
            if high >= low:
                points = [[r+t*(v-r) for r, v in zip(root, tip)] for t in (low, high)]
                clear = all(abs(p[1]-y)+radius <= ch["width"]/2+1e-9 and
                            abs(p[2]-channel_z)+radius <= ch["height"]/2+1e-9
                            for p in points)
                material_clear = all(abs(p[1])+radius < half[1] and
                                     abs(p[2])+radius < half[2] for p in points)
        shafts.append(dict(index=i, root_bale=root, tip_bale=tip,
                           depth_m=tip[0]+half[0],
                           overlap_m=segment_overlap(root, tip, half),
                           channel_clear=clear, material_clear=material_clear))
    return dict(shafts=shafts, depth_m=min(s["depth_m"] for s in shafts),
                min_overlap_m=min(s["overlap_m"] for s in shafts),
                all_channels_clear=all(s["channel_clear"] for s in shafts),
                all_material_clear=all(s["material_clear"] for s in shafts))
