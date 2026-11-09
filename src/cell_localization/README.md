# cell_localization (ROS 2)

Relocalizes an event camera inside a prior LiDAR map with **CELL**, from a coarse initial pose
guess. Built for **rosbag replay** (relocalization needs a perturbed/coarse guess). Edge-matching
pose refinement is **excluded** from the deployed node (small gain, high cost).

This package is a *consumer* of the CELL repo (`/home/panosk/CELL`): it imports CELL's `core/`
modules + compiled extensions (`visibility`, `correlation`, `mathutils`) and wraps inference. It does
not reimplement the method.

## Structure
```
cell_localization/
├── package.xml                     # manifest + deps (ament_python)
├── setup.py / setup.cfg            # build/install + console_scripts entry point
├── resource/cell_localization      # ament marker
├── config/params.yaml              # checkpoint, map, intrinsics, window, floor0, topics...
├── launch/localize.launch.py       # ros2 launch entry
└── cell_localization/
    ├── localizer_node.py           # the ROS node (subs/pubs/TF; the localization trigger)
    ├── cell_inference.py           # CELL forward: depth render + model + floor0 + PnP (no refine)
    └── event_representation.py     # raw events -> STC/Trail time-surface (SDK | SDK-free)
```

## Build & run
```bash
cd /home/panosk/cell_ros2_ws
colcon build --packages-select cell_localization
source install/setup.bash
ros2 launch cell_localization localize.launch.py
# in another terminal:
ros2 bag play <recording.bag>
```
The CELL conda env (torch + the three compiled extensions) must be importable by the node; point
`cell_root` in `params.yaml` at the CELL checkout.

## Node I/O
- **Sub** `pose_guess` (`geometry_msgs/PoseStamped`) — the coarse guess; **triggers** a localization.
- **Sub** `events` — raw events (message type TBD by sensor).
- **Sub** `camera_info` (`sensor_msgs/CameraInfo`) — intrinsics (or set fx/fy/cx/cy in params).
- **Pub** `/cell/pose` (`geometry_msgs/PoseStamped`) + TF `map -> camera`.

## Two layers: training/eval vs. deployment config
This package is **inference-only**. It does not train or evaluate — it is a *consumer* of a CELL
checkpoint. Keep the two layers distinct:

- **Training & evaluation stay in the CELL repo, unchanged.** Every training/eval flag
  (supervision options, per-scene crop/calib/max_depth, M3ED vs DSEC splits, selection variants, the
  event window, …) still lives in the CELL repo and is used exactly as before via its CLI.
  Deploying this node changes nothing about how you train or evaluate. Those knobs deliberately do
  **not** appear on the node — surfacing them would wrongly imply the node trains. To deploy a
  scene-specific model, train it in CELL, then point `checkpoint:` at the resulting weights.

- **Deployment is reconfigured per dataset/scene via ROS params, not by editing code.** Everything
  that differs across datasets and scenes is a parameter in `config/params.yaml` (override at launch
  with `ros2 launch cell_localization localize.launch.py params:=my_scene.yaml`):

  | Regime difference | Param |
  |---|---|
  | crop / max_depth per scene | `scene` (→ CELL `core.scene_config`) **or** explicit `crop` + `max_depth` |
  | intrinsics (M3ED / DSEC / new cam) | `CameraInfo` topic, or `fx/fy/cx/cy` override |
  | half vs full resolution (indoor/day `_half` vs night full) | `half_resolution` |
  | event window (50 vs 100 ms) | `event_window_us` |
  | selection rule | `select` (floor0 default) |
  | IFR iters (24 vs 12) | `iters` |
  | which trained model | `checkpoint` |
  | SDK vs SDK-free representation | `representation_backend` |

  Two ways to set geometry: **(A)** set `scene:` and the node pulls the *same* crop/max_depth that
  CELL eval uses from `core.scene_config.scene_geometry` (single source of truth — no drift); **(B)**
  set `scene: ""` and give `crop`/`max_depth`/`fx…` explicitly, for a dataset not yet in
  `scene_config` (e.g. new drone-space data before it's added).

## Open TODOs (need the camera/map provider)
1. **Event message type + decode** — `_subscribe_events` / `_on_events` in `localizer_node.py`.
   Prophesee/Metavision → `event_camera_msgs/EventPacket`; DVS/DAVIS → `dvs_msgs/EventArray`.
2. **STC/Trail backend** — `event_representation.py`: Metavision SDK (`sdk`) vs the numba
   reimplementation (`sdk_free`). The `sdk_free` path currently ships a shape-correct **stub**, not
   the trained representation.
3. **Prior map load + local crop** — `_get_local_map` (match `CELL/tools/m3ed/generate_local_maps.py`).
4. **Frame conventions** — pose-guess → `push_input` RT, and CELL Flow2Pose quat → ROS `map->camera`.
5. **Sensor resolution** — full-sensor rows/cols in `localizer_node` (currently M3ED's 1280×720).

See `../../DESIGN.md` for the full rationale.
