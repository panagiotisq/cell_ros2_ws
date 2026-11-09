# CELL — ROS 2 deployment design notes

**Date:** 2026-10-13
**Purpose:** context for building a ROS 2 package that runs CELL inference for
rosbag-replay relocalization. Captured from the design discussion before any ROS code exists.

**Locked decisions:** ROS 2 · rosbag-**replay** (not live), because relocalization needs a
perturbed / coarse initial pose guess · edge-matching pose refinement **excluded** from the
deployed node (small, regime-dependent gain, high cost) · sensors still TBD.

---

## 1. The C files — what they are, and what they mean for ROS

All C/C++/CUDA in CELL is **vendored dependencies**, none of it is the method's logic:

| Module | Language | What it does at inference |
|---|---|---|
| `core/visibility_package/` | C++/CUDA | Renders the LiDAR map → depth image at a given pose + hidden-point removal (`visibility.depth_image`, `visibility.visibility2`, `image_warp_index`). **Used every frame.** |
| `core/correlation_package/` | C++/CUDA | The cost-volume correlation layer inside the flow network. **Used every frame.** |
| `blender-mathutils/` | C (Python ext) | `mathutils.Quaternion`/`Matrix` — builds the pose rotation/translation matrices in `Data_preprocess`. **Used every frame.** |

Consequence for ROS: **nothing structural changes.** They compile to `.so` Python modules
(`python setup.py install`) exactly as today, and the node just `import visibility`,
`import mathutils`, etc. The only ROS-specific requirement is that the deployment PC's build
step compiles them against its CUDA. So "I also have C files" does **not** make this a C++ ROS
package — it stays a **Python (ament_python) package** depending on these prebuilt extensions.

## 2. Dropping edge refinement — and how it simplifies the package

Excluding edge-matching pose refinement from the ROS pipeline means the node's per-frame path is:

```
events → STC/Trail frame (ei) → model(di, ei) → flow + confidence
       → partial_light depth (di, from map@guess) → 2D–3D correspondences
       → floor0 selection → poselib PnP → pose
```

It lets us **leave out of the ROS package entirely**: `tools/pose_refine/`
(`refine.py`, `eval_refine.py`, `edge_extract.py`) and the edge-DT / anchor-sweep machinery.
Lower latency, fewer deps.

**Verified:** refinement is **eval-only, never in training.** `grep` confirms the edge-matching
pose refinement (`anchor_w`, `edge_conf_src`, 2D–3D edge alignment) lives *only* in
`tools/pose_refine/`; `main.py` (training) neither imports nor calls it. Other "refine" hits are
unrelated: `bench_runtime.py:time_refine` (the cost-analysis timer), `core/backbone.py`
"refinement trajectory" (RAFT-style **iterative flow refinement** inside the model),
`core/epropnp/levenberg_marquardt.py` "trust_region" (EPro-PnP's LM solver). So dropping
refinement from the node costs the deployment nothing training ever depended on, and the CELL
trainer (`main.py` + `scripts/train.sh`, full optimizer/scheduler/AMP/best_model loop) is intact.

## 3. The inputs — raw events vs. the event "frame"

Two different things both get loosely called "the event image":

- **(a) Raw events** — the stream of `(x, y, t, polarity)` tuples. This is what the sensor and the
  **rosbag actually contain.**
- **(b) The event representation / "event frame"** — the 2-channel STC/Trail **time-surface image**
  (`event_frame_00123.npy`). This is what the **model** consumes (the `ei` tensor in `model(di, ei)`).

In the paper/eval pipeline, **(a)→(b) was done offline** by `tools/m3ed/event2frame.py` using the
Metavision SDK (`SpatioTemporalContrastAlgorithm` + `TrailFilterAlgorithm`), and the model was fed
pre-rendered `.npy` frames. (In the reference setup the frames were produced upstream with a
licensed Metavision install; stage (a)→(b) was never run on an SDK-free machine.)

A rosbag gives raw events (a), **not** frames (b). So **the ROS node must do (a)→(b) online**, every
time it localizes — this is the one genuinely new piece of engineering. The node:

1. keeps a **rolling buffer of the last ~100 ms of events** (`event2frame.py` uses `time_window=100000` µs),
2. renders that buffer into the STC/Trail 2-channel time-surface = `ei`,
3. renders **depth `di`** from the prior LiDAR map at the current **coarse pose guess** (+ `partial_light` completion),
4. runs `model(di, ei)` → flow + confidence → correspondences → floor0 → PnP → **pose**.

### Node inputs

| Input | Source in deployment | From the bag? |
|---|---|---|
| **Raw events** | rosbag topic (event-camera driver) | yes |
| **Prior LiDAR map** | a `.pcd`/map file loaded once at startup | prior artifact, loaded once |
| **Coarse pose guess** | a pose topic (odometry / a previous estimate / an external initial guess) | the relocalization initial guess |
| **Camera intrinsics** | `CameraInfo` topic or a config file | config/topic |

The model outputs the **absolute camera pose in the map frame**; the guess enters only by deciding
which part of the map is rendered into `di` (must be within the ~5°/0.5 m basin — hence
replay with a coarse initial guess).

### The one real snag to decide: who builds the STC/Trail frame online?

`event2frame.py` calls the proprietary, license-gated **Metavision SDK**. For a live node, two options:

- **A — Metavision SDK on the drone-space PC.** If that machine has a licensed Metavision install
  (likely if the camera is a Prophesee with Metavision's own ROS driver), reuse the exact SDK filters
  → bit-identical representation to the paper.
- **B — SDK-free STC/Trail reimplementation.** Already reproduced the time-surface without the SDK
  (numba STC + latest-timestamp surface). Slight numerical differences, but the portable path.

Also: the event **message type** depends on the sensor. Prophesee's Metavision ROS 2 driver emits
`event_camera_msgs/EventPacket`; a DVS/DAVIS emits `dvs_msgs/EventArray`. The node's decode step
differs per driver — this is what "sensors unknown" blocks.

---

## Resulting package shape

ROS 2 package `cell_localization` (Python / ament_python) whose node wraps the **minimal deployment
path** (depth render + `model` + floor0 selection + poselib PnP, **no refinement**), adds the
**online event→STC/Trail step** on a rolling buffer, loads the **prior map** once, subscribes to
**events + coarse pose + CameraInfo**, and publishes a `PoseStamped` (+ TF `map→camera`). Depends on
the three compiled extensions (visibility / correlation / mathutils) built on the target PC, and on
either the Metavision SDK or the SDK-free STC/Trail backend.

## Layout decision

```
/home/panosk/
├── CELL/                      ← the method (unchanged, public MIT repo)
└── cell_ros2_ws/              ← this sibling ROS 2 workspace
    └── src/cell_localization/ ← the package (to be scaffolded)
```
Sibling (not inside CELL): keeps CELL's released tree free of colcon `build/ install/ log/` churn;
enforces the dependency direction (ROS node consumes CELL, never the reverse); lets CELL stay
independently releasable while the node pins a CELL version.

## Two things to lock before writing node code

1. **Representation backend:** Metavision SDK (A) vs. SDK-free STC/Trail (B) — depends on the deployment PC.
2. **Event message type:** needs the actual sensor (`event_camera_msgs` vs `dvs_msgs` vs other).

Everything else (package skeleton, map loading, depth render, model load, PnP, pose publishing) can
be scaffolded now.
