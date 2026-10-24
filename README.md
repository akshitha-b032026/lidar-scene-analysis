# Intelligent 3D LiDAR Scene Analysis

An interactive web app for exploring autonomous-driving LiDAR scenes. It loads point clouds and 3D object
annotations from the **nuScenes Mini** dataset, renders them in 3D in the browser, and shows the category,
position, size, distance and heading of every annotated object.

If the dataset is not installed, the app starts in **demo mode** with a synthetic street scene, so you can try it immediately.

## Features
- Interactive 3D viewer (orbit, pan, zoom) with an ego-vehicle marker and ground grid
- Scene picker, frame slider and play/pause through a scene's keyframes
- View switcher: full-screen LiDAR, full-screen camera, split, or inset (keys 1-4); hide the side panel (H) or go full screen (F)
- View the camera as 1, 2, 4 or all 6 cameras at once (front-left, front, front-right, back-left, back, back-right)
- Camera view (nuScenes data): all 6 cameras, with the 3D boxes and the LiDAR points (coloured by depth) projected onto each image
- Annotation checks: flags boxes with no/few LiDAR points, implausible sizes, floating or sunken boxes, and overlaps
- Colour points by height, intensity or distance; adjustable point size and max range
- 3D bounding boxes with heading tick, colour-coded by category group
- Filter by category group; click a box (or a list row) to inspect it
- Object details: category, position, size, distance, heading, LiDAR point count, visibility
- Objects list sorted by distance from the ego vehicle

## Project structure
```
lidar-scene-analysis/
├── backend/
│   ├── app.py        # FastAPI app: REST API + serves the frontend
│   └── sources.py    # Data sources: NuScenesSource and DemoSource (same interface)
├── frontend/
│   ├── index.html    # Layout
│   ├── style.css     # Styling
│   └── app.js        # Three.js viewer, filters, selection, API calls
├── data/             # Put the nuScenes dataset here (data/nuscenes/v1.0-mini)
├── requirements.txt
└── README.md
```

## Setup
Requires Python 3.9+.

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### Get the data (optional but recommended)
1. Create a free account at https://www.nuscenes.org and download **"Mini"** (v1.0-mini, about 4 GB).
2. Extract it so the layout is:
   ```
   data/nuscenes/
   ├── maps/  samples/  sweeps/
   └── v1.0-mini/
   ```
   Or point to it elsewhere: `export NUSCENES_DATAROOT=/path/to/nuscenes`

### Run
```bash
uvicorn backend.app:app --reload
```
Open http://127.0.0.1:8000. The header of the sidebar shows whether you are on real data or demo mode.
Interactive API docs are at http://127.0.0.1:8000/docs.

## API
| Endpoint | Returns |
|---|---|
| `GET /api/info` | `{mode: "nuscenes" \| "demo"}` |
| `GET /api/scenes` | Scenes: token, name, description, sample count |
| `GET /api/scenes/{scene}/samples` | Keyframes of a scene, in order |
| `GET /api/samples/{sample}/points?max_points=60000&max_range=80` | Binary little-endian float32, N×4 (x, y, z, intensity) |
| `GET /api/samples/{sample}/boxes` | Annotated boxes with category, centre, size, yaw, distance, corners |
| `GET /api/samples/{sample}/cameras` | Camera names available (empty in demo mode) |
| `GET /api/samples/{sample}/image/{cam}` | JPEG of that camera |
| `GET /api/samples/{sample}/camera_points/{cam}` | LiDAR points projected into the image: binary float32 N×3 (u, v, depth) |
| `GET /api/samples/{sample}/projection/{cam}` | Boxes projected to pixel coordinates (8 corners each) |
| `GET /api/samples/{sample}/checks` | Annotation-quality flags per box, with severity and reasons |
| `GET /api/samples/{sample}/summary` | Object counts per category group |

## Design notes
- **Coordinate frame:** everything is sent in the ego-vehicle frame (x forward, y left, z up). Points are moved
  from the LIDAR_TOP sensor frame using the calibrated-sensor record; boxes are moved from the global frame using the ego pose.
- **Box convention:** `size` is `[width, length, height]` as in nuScenes; `yaw` is the rotation about z, and corners 0–1 are the front edge.
- **Performance:** a nuScenes sweep has about 30k points, which is sent as raw binary instead of JSON.
  The server can also subsample (`max_points`) and range-filter.
- **Modularity:** the API only talks to the `source` interface (`scenes`, `samples`, `points`, `boxes`).
  Adding a new dataset, or an analysis step, means adding a class or a route, not changing the viewer.

## Roadmap (future extensions)
- ML-based analysis: run a 3D detector (e.g. PointPillars) and compare predictions with ground truth
- Annotation checking: implemented as rule-based checks in `backend/analysis.py` (tests: `python -m tests.test_analysis`); next step is learned checks
- Anomaly detection on point density and object dimensions
- Multi-frame analysis: object tracks by instance token, speed and trajectory estimation
- Camera + LiDAR fusion: project boxes onto the camera images and colour points from RGB

Suggested next step: add more rules or a learned model to `backend/analysis.py`; the API and UI already show whatever flags it returns.

## Troubleshooting
- **"Demo mode" but I installed the dataset:** check that `data/nuscenes/v1.0-mini/` exists, or set `NUSCENES_DATAROOT`.
- **Blank 3D view:** the page loads Three.js and fonts from a CDN, so it needs internet access on first load.
- **Slow on large ranges:** lower the max-range slider; the server then sends fewer points.

## Licence and data
Code: MIT. nuScenes data is © Motional and is licensed separately for non-commercial use; see https://www.nuscenes.org/terms-of-use.
