# On-site checklist (drone space)

Everything here can **only** be done on-site. Keep this open while you work. The reconstruction
(COLMAP) is handled off-site on the GPU machine — don't spend on-site time on it.

## Read first — two rules that silently ruin the data if broken
1. **Keep the room physically unchanged** between the reconstruction video and ALL rosbag
   recordings. The map must match what the events see; if objects move, rendered depth won't line up.
2. **Do NOT recalibrate / re-origin the mocap** during the session. Everything — all bags AND the
   control points — must share **one** mocap world frame.

---

## 1. Record the 3 rosbags (2 train + 1 eval)
- Use the existing ROS acquisition setup (don't modify its scripts).
- Each bag must contain **both**: the **event stream** + the **mocap rigidbody pose**, time-synced.
- Make the 3 motions **distinct** (like M3ED flight_1/2/3): vary the trajectory and coverage; the
  **eval** sequence should be clearly different from the two train ones.
- **Verify each bag right after recording** (`rosbag info <bag>`): both topics present, sane
  duration, message counts non-zero for both streams. (The PC3 Claude can do this for you.)
- Name clearly: `seq_train1`, `seq_train2`, `seq_eval`.

## 2. Control points for mocap alignment — DO NOT SKIP
Goal: **≥6 points** with known **mocap-world XYZ**, each **identifiable in the reconstruction**.
This is what lets the map be placed into the mocap frame later. Easy to forget, impossible to
recover once you leave.

Procedure:
- Pick 6–10 **sharp, distinct** features (corners of a table/box/equipment, wall-floor junctions).
- **Spread them across the room AND at varied heights** — not all on the floor, not coplanar, or the
  vertical scale/rotation is poorly constrained.
- Measure each point's **mocap-world XYZ** (touch it with a mocap-tracked probe/wand tip, or place a
  marker there and read its mocap position).
- Record a **precise description** per point so it can be located in the cloud.

Template (fill in):

| ID | Description (precise) | mocap X | mocap Y | mocap Z |
|----|-----------------------|---------|---------|---------|
| P1 | e.g. NE top corner of metal table |  |  |  |
| P2 | e.g. wall-floor-wall corner, SW |  |  |  |
| P3 | ... (vary height!) |  |  |  |
| P4 |  |  |  |  |
| P5 |  |  |  |  |
| P6 |  |  |  |  |

**Gold-standard option (≈20 min, tighter alignment):** stick ~6 distinct markers (tape crosses /
printed tags) around the room at varied heights, measure each marker's mocap XYZ, then shoot **one
more 30–60s video that includes them**. Markers are unambiguous in both the cloud and your notes, so
the fit is far better than matching natural features by eye. The extra clip gets reconstructed
off-site.

## 3. Get from the colleague (gates the later steps)
- [x] **Event-camera intrinsics** — see `CALIBRATION.md` / `calib/intrinsics.json`.
- [x] **Rigidbody → camera extrinsic** — see `CALIBRATION.md` / `calib/T_body_camera.txt`.
      Convention confirmed by the owner: **camera → body** (`T_world_camera = T_world_body · T_body_camera`).
- [x] **Time sync** — confirmed from an existing PC-C test bag: events and mocap poses share one
      ROS time base (seconds), no offset. See `CALIBRATION.md`.
- [x] **Event topic name + message type** — `/prophesee/camera/cd_events_buffer`
      (`prophesee_event_msgs/EventArray`); mocap on `/mocap_node/evk4/pose`
      (`geometry_msgs/PoseStamped`). See `CALIBRATION.md`.

## 4. Before you leave — confirm you have
- [ ] 3 rosbags, each verified (both streams present, time-synced)
- [ ] ≥6 control points with mocap XYZ + descriptions (varied heights)
- [ ] (optional) the marker video
- [ ] intrinsics, extrinsic, time-sync, topic/message-type answers
- [ ] a way to transfer all of it to the GPU/lab machine

Then bring everything to the lab machine — we solve the alignment (7-DOF similarity over the control
points) and build the CELL sequences there.
