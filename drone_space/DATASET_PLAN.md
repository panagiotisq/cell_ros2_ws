# Drone-space dataset — plan

Goal: capture a custom dataset to **train + evaluate** the CELL event-camera relocalization method on
real data — **2 training sequences + 1 eval/test sequence**, structured like the M3ED indoor-flight
set. This dataset later validates the `cell_localization` ROS 2 package in this repo.

## The pieces
- **Event camera** (Prophesee family), recorded to **rosbags** on an existing ROS 1 acquisition setup.
- A **motion-capture** system giving ground-truth 6-DOF pose of a tracked **rigidbody**; a known
  **rigidbody→camera extrinsic** converts it to the camera pose. The mocap world frame is metric.
- A **3D reconstruction of the room** (no depth sensor available), built with **COLMAP** from a phone
  video — this is the **prior map** depth is rendered from (the LiDAR-map analogue in M3ED).

## Pipeline
1. **Reconstruct** the room → dense point cloud (COLMAP; see `PC3_RECON_STEPS.md`). Done on-site so a
   bad capture can be re-shot.
2. **Align + metric-scale** the cloud into the **mocap world frame** via a 7-DOF similarity fit
   (Umeyama) over ≥4 measured control points. Done on the GPU/lab machine.
3. **Record** the rosbags (events + mocap poses, time-synced on a common clock).
4. **Build CELL sequences:** per GT pose timestamp, camera world pose = `mocap_rigidbody_pose ∘
   extrinsic`; render the local map/depth at that pose; build the 2-channel **STC/Trail** event frame
   from the ~100 ms window before it; save in the CELL dataloader layout (match an existing M3ED
   sequence as the template). The events→frame step uses the **SDK-free** STC/Trail implementation
   (no proprietary SDK needed), shared with the ROS package's `event_representation` backend.

## Still to confirm
- **Event-camera intrinsics** (focal / principal point / distortion) — already calibrated?
- The **extrinsic** value + its convention.
- **Time sync** between events and mocap poses in the bag (same clock? µs?).
- Reconstruction **scale reference** (control points vs a known-size object placed in the scene).

## Where work happens
- **On-site PC (ROS 1):** record rosbags; run COLMAP reconstruction (see `PC3_RECON_STEPS.md`).
- **GPU/lab machine:** mocap alignment, sequence generation, training, eval. Transfer the dense cloud
  + rosbags + control-point coordinates + intrinsics/extrinsic there when ready.
