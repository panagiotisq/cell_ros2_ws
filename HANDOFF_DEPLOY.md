# HANDOFF — CELL ROS2 deployment (continue on the deployment machine)

**Date:** 2026-10-15. Read this first on the ROS2/lab computer, then `DESIGN.md` (full rationale)
and `src/cell_localization/README.md` (build/run + the two-layer config explanation).

## Why this file exists
The scaffold and all the *design* decisions are done. Everything still open is **empirical** — it
can only be resolved on the real machine (what's installed, the real event message type, whether a
bag round-trips, whether `sdk_free` matches the trained representation). Don't reason about these;
run the discovery commands below on the actual ROS2 computer.

---

## Locked decisions
- **ROS2**, **rosbag-replay** (not live) — relocalization needs a coarse/perturbed initial pose guess.
- **Edge-based pose refinement is excluded** from the deployed node (little gain, high cost).
- **Public deliverable**, not a one-off → build general; **our setup is just the first reference
  example profile**, not the only thing it can do. Everything regime-specific is a ROS param.
- **`sdk_free` is the default** representation backend (portable, no Metavision license). `sdk`
  (bit-identical Metavision STC/Trail) stays an opt-in for exact reproduction.
- Camera is a **Prophesee** (Metavision family) → ROS2 driver message type is most likely
  `event_camera_msgs/EventPacket` (Bernd Pfrommer's `metavision_driver`). **Confirm from a real bag.**

## Drone-space hardware situation (reported 2026-10-15)
- One computer: **old Ubuntu + ROS1**, has the **proprietary Metavision SDK**.
- Other computer: **ROS2**, "hasn't the SDK" — **ambiguous**, must disambiguate on the machine:
  - (A) lacks only the *proprietary CV filters* (STC/Trail) but has the driver → can capture raw
    events directly → **best case, run everything on ROS2 with `sdk_free`**.
  - (B) lacks the *driver entirely* (no OpenEB) → either install free OpenEB + `metavision_driver`,
    or use bag-replay (record on the ROS1/SDK machine, replay on ROS2).
- **Conclusion either way:** the ROS2 computer is the home for the node. The proprietary SDK is
  never on the deployment path (we use `sdk_free`). For **bag-replay**, the replay machine needs
  neither the driver nor the SDK — only `event_camera_msgs` (message defs) + `event_camera_codecs`
  (decoder), both free.

## The two Metavision layers (don't conflate — this is what "SDK" ambiguity is about)
| Layer | Package(s) | Licensed? | Purpose |
|---|---|---|---|
| Driver / HAL | OpenEB (`metavision_hal`,`metavision_core`) + `metavision_driver` (ROS2) | **Free** | capture/record raw events |
| CV filters | `metavision_sdk_cv` (STC/Trail) | **Licensed** | the exact offline representation (we replace with `sdk_free`) |

---

## What already exists (on the panosk dev machine — move it to the target)
- **CELL repo:** `/home/panosk/CELL` (the method; the ROS package imports from it).
- **Scaffold:** `/home/panosk/cell_ros2_ws/src/cell_localization/` — a complete, compiling
  ament_python package. Modules:
  - `localizer_node.py` — ROS node: subs (events STUB, pose_guess=trigger, camera_info), pub
    PoseStamped + TF. Real CELL path wired. Stubs: `_subscribe_events`/`_on_events`, `_get_local_map`.
  - `cell_inference.py` — CELL forward ported verbatim from `eval_full.py`: depth render via
    `push_input` → `partial_light` completion → `model(di, ei)` → floor0 selection → poselib PnP.
    **No refinement.** This part is real, not stubbed.
  - `event_representation.py` — raw events → STC/Trail time-surface. `sdk_free` path is a
    **shape-correct STUB** (plain latest-ts surface, no STC/Trail denoising). `sdk` path raises.
  - `config/params.yaml`, `launch/localize.launch.py`, `package.xml`, `setup.py` — standard.

### Get the code onto the target ROS2 machine
Both the CELL repo and `cell_ros2_ws` must be present. Use git (push from here, clone there) or
rsync over Tailscale. Then point `cell_root` in `params.yaml` at the CELL checkout, and make the
CELL conda env (torch + compiled extensions `visibility`/`correlation`/`mathutils`) importable.

---

## RUN THESE ON THE ROS2 MACHINE FIRST (empirical discovery)
```bash
# 1. ROS2 distro + what's installed
printenv ROS_DISTRO
ros2 pkg list | grep -iE 'event_camera|metavision|dvs|prophesee'
python3 -c "import metavision_hal" 2>&1 | head   # is OpenEB/driver present? (layer A vs B)

# 2. Can it see the camera live? (if a camera is attached)
ros2 topic list
ros2 topic echo <events_topic> --once            # what message TYPE prints?

# 3. Inspect a recorded bag from the camera/map provider (the single most useful artifact)
ros2 bag info <path/to/bag>                       # topics + message types + duration
#   -> confirms event message type (EventPacket vs dvs_msgs/EventArray vs other)
#   -> confirms there's a pose-guess topic and/or camera_info

# 4. Test sdk_free fidelity (THE real technical risk)
#   Decode a slice of events from the bag, run event_representation._build_sdk_free,
#   and compare to CELL/tools/m3ed/event2frame.py (SDK offline) on the SAME events.
#   The pose accuracy depends on this matching the trained representation.
```

## Still to confirm (from the camera/map provider)
1. **A sample recorded bag** (confirms msg type, enables sdk_free test + ROS1→ROS2 conversion test).
2. Does the ROS2 computer lack only the CV filters, or the driver too (layer A vs B)? (`ros2 topic echo` answers it.)
3. Any **online/streaming STC/Trail** code that already exists (beats rebuilding from offline event2frame.py).
4. **Where the coarse pose guess comes from** at deployment + its frame (map→camera? map→body?).
5. The **prior LiDAR map**: format, coordinate convention (match `generate_local_maps.py`), crop radius.
6. **Intrinsics + camera↔LiDAR extrinsics**, and whether `CameraInfo` is published.
7. **Time sync** between events and pose guesses (same clock? µs?).

---

## First implementation targets once on the machine (in order)
1. **`_subscribe_events` / `_on_events`** (`localizer_node.py`) for the confirmed message type →
   decode to `(x, y, t_µs, polarity)`. Support both `event_camera_msgs/EventPacket` (via
   `event_camera_codecs`) and `dvs_msgs/EventArray`, selected by param (public-deliverable goal).
2. **Real `sdk_free` STC/Trail** (`event_representation._build_sdk_free`) — port the numba
   reimplementation (earlier note: "latest-ts time-surface via numba STC + `maximum.at`"); validate
   against `event2frame.py` on the same events.
3. **`_get_local_map`** — load the prior map once, crop a local submap around the guess, match
   `CELL/tools/m3ed/generate_local_maps.py` conventions.
4. **Frame conventions** — pose-guess → `push_input` RT; CELL Flow2Pose quat → ROS `map→camera`.
5. **Sensor resolution** — set full-sensor rows/cols in `localizer_node` (currently M3ED 1280×720).

## Then
- Ship a worked **example profile** (`examples/falcon_m3ed/` + a sample bag + README walkthrough) so
  a newcomer runs it end-to-end before adapting their own sensor/map/topics.
