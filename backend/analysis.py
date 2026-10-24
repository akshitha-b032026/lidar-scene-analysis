"""Rule-based annotation checks. Input: boxes (see sources.make_box) and points (N,4), both in the ego frame.

Each check is a simple, explainable rule, so a flag can be understood and challenged by a person.
"""
import numpy as np

# category prefix -> ((min, max) length, width, height) in metres
SIZE_LIMITS = {
    "human.pedestrian": ((0.2, 1.2), (0.2, 1.2), (0.8, 2.3)),
    "vehicle.bicycle": ((0.8, 2.4), (0.3, 1.2), (0.8, 2.2)),
    "vehicle.motorcycle": ((1.0, 2.8), (0.4, 1.4), (0.8, 2.2)),
    "vehicle.car": ((3.0, 6.5), (1.4, 2.5), (1.0, 2.5)),
    "vehicle.truck": ((4.0, 14.0), (1.8, 3.2), (1.8, 4.5)),
    "vehicle.bus": ((6.0, 20.0), (2.0, 3.2), (2.4, 4.5)),
    "movable_object.trafficcone": ((0.1, 0.8), (0.1, 0.8), (0.3, 1.2)),
}
FEW_POINTS, NEAR_M = 5, 30           # "few points" only matters for objects closer than NEAR_M
FLOAT_GAP, SINK_GAP = 1.0, 0.8       # metres between box bottom and local ground


def _limits(cat):
    for prefix in sorted(SIZE_LIMITS, key=len, reverse=True):
        if cat.startswith(prefix):
            return SIZE_LIMITS[prefix]
    return None


def check_sample(boxes, pts):
    flags = {}

    def add(i, msg, high=False):
        f = flags.setdefault(boxes[i]["id"], {"id": boxes[i]["id"], "severity": "warn", "issues": []})
        f["issues"].append(msg)
        if high:
            f["severity"] = "high"

    for i, b in enumerate(boxes):
        w, l, h = b["size"]
        cx, cy, cz = b["center"]
        n = b["num_lidar_pts"]
        if n == 0:
            add(i, "No LiDAR points inside the box", high=True)
        elif n < FEW_POINTS and b["distance"] < NEAR_M:
            add(i, f"Only {n} LiDAR points at {b['distance']:.0f} m")
        lim = _limits(b["category"])
        if lim:
            for name, v, (lo, hi) in zip(("length", "width", "height"), (l, w, h), lim):
                if not lo <= v <= hi:
                    add(i, f"Unusual {name} {v:.2f} m (expected {lo}-{hi} m)")
        r = max(l, w) / 2 + 1.5                           # local ground = low percentile of nearby points
        near = pts[(np.abs(pts[:, 0] - cx) < r) & (np.abs(pts[:, 1] - cy) < r)]
        if len(near) >= 15:
            gap = (cz - h / 2) - np.percentile(near[:, 2], 10)
            if gap > FLOAT_GAP:
                add(i, f"Box floats {gap:.1f} m above the ground", high=True)
            elif gap < -SINK_GAP:
                add(i, f"Box sinks {-gap:.1f} m below the ground", high=True)

    for i in range(len(boxes)):                           # overlapping boxes
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            d = np.hypot(a["center"][0] - b["center"][0], a["center"][1] - b["center"][1])
            reach = 0.5 * min(max(a["size"][:2]), max(b["size"][:2]))
            dz = abs(a["center"][2] - b["center"][2])
            if d < reach and dz < 0.5 * (a["size"][2] + b["size"][2]) / 2:
                add(i, f"Overlaps another box ({b['category']})", high=True)
                add(j, f"Overlaps another box ({a['category']})", high=True)
    return list(flags.values())
