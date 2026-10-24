"""Data sources. Both expose the same interface so the API never cares which is used.

    scenes() -> list[dict]          samples(scene_token) -> list[dict]
    points(sample_token) -> (N,4) float32 array [x, y, z, intensity] in the EGO frame
    boxes(sample_token) -> list[dict]  (see make_box)
"""
import math
import os
from pathlib import Path

import numpy as np


def corners(center, size, yaw):
    """8 box corners (ego frame). size = [width, length, height]; corners 0-3 top, 4-7 bottom, 0-1 = front edge."""
    w, l, h = size
    x = np.array([1, 1, -1, -1, 1, 1, -1, -1]) * l / 2
    y = np.array([1, -1, -1, 1, 1, -1, -1, 1]) * w / 2
    z = np.array([1, 1, 1, 1, -1, -1, -1, -1]) * h / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([c * x - s * y + center[0], s * x + c * y + center[1], z + center[2]], 1).round(3).tolist()


def make_box(id_, category, center, size, yaw, n_pts, visibility):
    center = [round(float(v), 3) for v in center]
    return {
        "id": id_,
        "category": category,
        "center": center,
        "size": [round(float(v), 3) for v in size],  # width, length, height (m)
        "yaw_deg": round(math.degrees(yaw), 1),
        "yaw": round(float(yaw), 4),
        "distance": round(math.hypot(center[0], center[1]), 2),
        "num_lidar_pts": int(n_pts),
        "visibility": visibility,
        "corners": corners(center, size, yaw),
    }


class NuScenesSource:
    name = "nuscenes"

    def __init__(self, dataroot, version):
        from nuscenes.nuscenes import NuScenes

        self.nusc = NuScenes(version=version, dataroot=dataroot, verbose=False)

    def scenes(self):
        return [dict(token=s["token"], name=s["name"], description=s["description"],
                     n_samples=s["nbr_samples"]) for s in self.nusc.scene]

    def samples(self, scene_token):
        t = self.nusc.get("scene", scene_token)["first_sample_token"]
        out = []
        while t:
            s = self.nusc.get("sample", t)
            out.append(dict(token=t, index=len(out), timestamp=s["timestamp"]))
            t = s["next"]
        return out

    def points(self, token):
        from nuscenes.utils.data_classes import LidarPointCloud
        from pyquaternion import Quaternion

        nusc = self.nusc
        sd = nusc.get("sample_data", nusc.get("sample", token)["data"]["LIDAR_TOP"])
        pc = LidarPointCloud.from_file(os.path.join(nusc.dataroot, sd["filename"]))
        cs = nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])
        pc.rotate(Quaternion(cs["rotation"]).rotation_matrix)  # sensor -> ego
        pc.translate(np.array(cs["translation"]))
        return pc.points[:4].T.astype(np.float32)

    def boxes(self, token):
        from pyquaternion import Quaternion

        nusc = self.nusc
        sample = nusc.get("sample", token)
        sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        pose = nusc.get("ego_pose", sd["ego_pose_token"])
        out = []
        for a in sample["anns"]:
            ann = nusc.get("sample_annotation", a)
            b = nusc.get_box(a)  # global frame
            b.translate(-np.array(pose["translation"]))  # global -> ego
            b.rotate(Quaternion(pose["rotation"]).inverse)
            inst = nusc.get("instance", ann["instance_token"])
            cat = nusc.get("category", inst["category_token"])["name"]
            vis = nusc.get("visibility", ann["visibility_token"])["level"] if ann["visibility_token"] else "n/a"
            out.append(make_box(a[:8], cat, b.center, b.wlh, b.orientation.yaw_pitch_roll[0],
                                ann["num_lidar_pts"], vis))
        return out

    CAMS = ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"]

    def image_path(self, token, cam):
        sd = self.nusc.get("sample_data", self.nusc.get("sample", token)["data"][cam])
        return os.path.join(self.nusc.dataroot, sd["filename"])

    def project(self, token, cam):
        """Project every box's 8 corners into a camera image. Unprojectable corners are None."""
        from pyquaternion import Quaternion

        nusc = self.nusc
        sample = nusc.get("sample", token)
        lid = nusc.get("ego_pose", nusc.get("sample_data", sample["data"]["LIDAR_TOP"])["ego_pose_token"])
        csd = nusc.get("sample_data", sample["data"][cam])
        cpose = nusc.get("ego_pose", csd["ego_pose_token"])
        cs = nusc.get("calibrated_sensor", csd["calibrated_sensor_token"])
        K = np.array(cs["camera_intrinsic"])
        Rl, tl = Quaternion(lid["rotation"]).rotation_matrix, np.array(lid["translation"])[:, None]
        Rc, tc = Quaternion(cpose["rotation"]).rotation_matrix, np.array(cpose["translation"])[:, None]
        Rs, ts = Quaternion(cs["rotation"]).rotation_matrix, np.array(cs["translation"])[:, None]
        out = []
        for b in self.boxes(token):
            p = np.array(b["corners"]).T                 # 3x8, ego frame at lidar time
            p = Rl @ p + tl                              # -> global
            p = Rc.T @ (p - tc)                          # -> ego frame at camera time
            p = Rs.T @ (p - ts)                          # -> camera frame
            ok = p[2] > 0.1
            if not ok.any():
                continue
            uv = (K @ p)[:2] / np.where(ok, p[2], 1)
            out.append({"id": b["id"], "category": b["category"],
                        "pts": [[round(float(uv[0, i]), 1), round(float(uv[1, i]), 1)] if ok[i] else None
                                for i in range(8)]})
        return {"width": csd.get("width", 1600), "height": csd.get("height", 900), "boxes": out}

    def project_points(self, token, cam):
        """LiDAR points that land inside a camera image -> (N, 3) float32 [u, v, depth_m]."""
        from pyquaternion import Quaternion

        nusc = self.nusc
        sample = nusc.get("sample", token)
        lid = nusc.get("ego_pose", nusc.get("sample_data", sample["data"]["LIDAR_TOP"])["ego_pose_token"])
        csd = nusc.get("sample_data", sample["data"][cam])
        cpose = nusc.get("ego_pose", csd["ego_pose_token"])
        cs = nusc.get("calibrated_sensor", csd["calibrated_sensor_token"])
        K = np.array(cs["camera_intrinsic"])
        p = self.points(token)[:, :3].T.astype(np.float64)                      # ego frame at lidar time
        p = Quaternion(lid["rotation"]).rotation_matrix @ p + np.array(lid["translation"])[:, None]   # -> global
        p = Quaternion(cpose["rotation"]).rotation_matrix.T @ (p - np.array(cpose["translation"])[:, None])
        p = Quaternion(cs["rotation"]).rotation_matrix.T @ (p - np.array(cs["translation"])[:, None])   # -> camera
        w, h = csd.get("width", 1600), csd.get("height", 900)
        ok = p[2] > 1.0
        uv = (K @ p)[:2] / np.where(ok, p[2], 1)
        ok &= (uv[0] >= 0) & (uv[0] < w) & (uv[1] >= 0) & (uv[1] < h)
        return np.stack([uv[0, ok], uv[1, ok], p[2, ok]], 1).astype(np.float32)


class DemoSource:
    """Synthetic street scene so the app runs without the dataset."""
    name = "demo"
    N = 24
    # category, (w, l, h), start (x, y), velocity (vx, vy) m/s, yaw
    OBJS = [
        ("vehicle.car", (1.9, 4.5, 1.6), (14, 0), (-1.0, 0), 0.0),
        ("vehicle.car", (1.9, 4.6, 1.6), (38, 4), (-7.0, 0), math.pi),
        ("vehicle.truck", (2.6, 8.0, 3.2), (-22, -4), (3.0, 0), 0.0),
        ("human.pedestrian.adult", (0.7, 0.7, 1.75), (8, -9), (0, 1.3), math.pi / 2),
        ("vehicle.bicycle", (0.6, 1.8, 1.4), (-10, 6), (2.5, 0), 0.0),
        ("movable_object.trafficcone", (0.4, 0.4, 0.7), (6, 7), (0, 0), 0.0),
        ("movable_object.barrier", (2.5, 0.6, 1.0), (22, -8), (0, 0), 0.0),
    ]

    def scenes(self):
        return [dict(token="demo-scene", name="demo-scene-01", n_samples=self.N,
                     description="Synthetic street scene (nuScenes data not found)")]

    def samples(self, scene_token):
        return [dict(token=f"demo-{i}", index=i, timestamp=i * 500000) for i in range(self.N)]

    def _objs(self, i):
        t = i * 0.5
        return [(c, s, (p[0] + v[0] * t, p[1] + v[1] * t), yaw) for c, s, p, v, yaw in self.OBJS]

    def points(self, token):
        i = int(token.split("-")[1])
        rng = np.random.default_rng(i)
        n = 22000
        r, th = 2 + 48 * np.sqrt(rng.random(n)), rng.uniform(0, 2 * np.pi, n)
        parts = [np.c_[r * np.cos(th), r * np.sin(th), -1.8 + rng.normal(0, .03, n), rng.uniform(0, 30, n)]]
        m = 7000
        for side in (-18, 18):  # building facades
            parts.append(np.c_[rng.uniform(-50, 50, m), side + rng.normal(0, .05, m),
                               rng.uniform(-1.8, 9, m), rng.uniform(5, 60, m)])
        for _, (w, l, h), (x, y), yaw in self._objs(i):
            k = 350
            p = rng.uniform(-.5, .5, (k, 3))
            ax = rng.integers(0, 3, k)
            p[np.arange(k), ax] = np.sign(rng.random(k) - .5) * .5
            p *= (l, w, h)
            c, s = math.cos(yaw), math.sin(yaw)
            parts.append(np.c_[c * p[:, 0] - s * p[:, 1] + x, s * p[:, 0] + c * p[:, 1] + y,
                               p[:, 2] - 1.8 + h / 2, rng.uniform(20, 100, k)])
        return np.vstack(parts).astype(np.float32)

    def boxes(self, token):
        i = int(token.split("-")[1])
        return [make_box(f"demo{i:02d}{j}", c, (x, y, -1.8 + s[2] / 2), s, yaw, 350, "4")
                for j, (c, s, (x, y), yaw) in enumerate(self._objs(i))]


def load_source():
    root = Path(os.environ.get("NUSCENES_DATAROOT", Path(__file__).resolve().parent.parent / "data" / "nuscenes"))
    version = os.environ.get("NUSCENES_VERSION", "v1.0-mini")
    if (root / version).exists():
        return NuScenesSource(str(root), version)
    print(f"[!] nuScenes not found at {root / version} -> running in DEMO mode")
    return DemoSource()
