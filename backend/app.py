"""FastAPI backend. Run from the project root:  uvicorn backend.app:app --reload"""
from collections import Counter
from pathlib import Path

import numpy as np
from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from .analysis import check_sample
from .sources import load_source

app = FastAPI(title="Intelligent 3D LiDAR Scene Analysis")


@app.middleware("http")
async def no_cache(request, call_next):
    """Always serve fresh frontend files so an updated UI never hides behind the browser cache."""
    resp = await call_next(request)
    if not request.url.path.startswith("/api/samples"):
        resp.headers["Cache-Control"] = "no-store"
    return resp
src = load_source()


@app.get("/api/info")
def info():
    return {"mode": src.name}


@app.get("/api/scenes")
def scenes():
    return src.scenes()


@app.get("/api/scenes/{scene_token}/samples")
def samples(scene_token: str):
    try:
        return src.samples(scene_token)
    except Exception:
        raise HTTPException(404, "Unknown scene")


@app.get("/api/samples/{token}/points")
def points(token: str, max_points: int = Query(60000, ge=1000, le=300000),
           max_range: float = Query(80, ge=5, le=200)):
    """Binary float32 array, N x 4 (x, y, z, intensity), ego frame, row-major."""
    try:
        pts = src.points(token)
    except Exception:
        raise HTTPException(404, "Unknown sample")
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= max_range]
    total = len(pts)
    if total > max_points:
        pts = pts[np.random.default_rng(0).choice(total, max_points, replace=False)]
    return Response(np.ascontiguousarray(pts, dtype="<f4").tobytes(),
                    media_type="application/octet-stream", headers={"X-Total-Points": str(total)})


@app.get("/api/samples/{token}/boxes")
def boxes(token: str):
    try:
        return src.boxes(token)
    except Exception:
        raise HTTPException(404, "Unknown sample")


@app.get("/api/samples/{token}/cameras")
def cameras(token: str):
    return list(getattr(src, "CAMS", []))  # empty in demo mode


def _check_cam(cam):
    if cam not in getattr(src, "CAMS", []):
        raise HTTPException(404, "Camera images are only available with nuScenes data")


@app.get("/api/samples/{token}/image/{cam}")
def image(token: str, cam: str):
    _check_cam(cam)
    return FileResponse(src.image_path(token, cam), media_type="image/jpeg")


@app.get("/api/samples/{token}/projection/{cam}")
def projection(token: str, cam: str):
    _check_cam(cam)
    return src.project(token, cam)


@app.get("/api/samples/{token}/checks")
def checks(token: str):
    """Annotation-quality flags: [{id, severity: 'warn'|'high', issues: [str]}]"""
    try:
        return check_sample(src.boxes(token), src.points(token))
    except Exception:
        raise HTTPException(404, "Unknown sample")


@app.get("/api/samples/{token}/camera_points/{cam}")
def camera_points(token: str, cam: str, max_points: int = Query(12000, ge=1000, le=60000)):
    """Binary float32 N x 3 (u, v, depth): LiDAR points projected onto the camera image."""
    _check_cam(cam)
    a = src.project_points(token, cam)
    if len(a) > max_points:
        a = a[np.random.default_rng(0).choice(len(a), max_points, replace=False)]
    return Response(np.ascontiguousarray(a, dtype="<f4").tobytes(), media_type="application/octet-stream")


@app.get("/api/samples/{token}/summary")
def summary(token: str):
    b = boxes(token)
    return {"objects": len(b), "by_category": Counter(x["category"].split(".")[0] for x in b),
            "nearest": min(b, key=lambda x: x["distance"])["category"] if b else None}


app.mount("/", StaticFiles(directory=Path(__file__).resolve().parent.parent / "frontend", html=True), name="ui")
