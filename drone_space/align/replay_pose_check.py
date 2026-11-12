#!/usr/bin/env python3
"""
Alignment visual validation from a RECORDED bag -- no live ROS master needed.

Reads /mocap_node/evk4/pose directly from a rosbag file and animates the camera marker
(position + orientation, composed via T_body_camera) through the aligned point cloud at the
bag's own pace. Use this to visually confirm seq1..seq4's ground-truth poses line up with the
reconstruction -- same idea as live_pose_check.py, but driven by a bag instead of a live topic,
so it doesn't need roscore / rosbag play running.

Usage:
    python3 replay_pose_check.py /path/to/seq1.bag [--speed 1.0] [--ply ../map/fused_clean_mocap.ply]

Requires: open3d, numpy, rosbag + the prophesee_event_msgs/geometry_msgs Python message
definitions on PYTHONPATH (i.e. a sourced ROS1 workspace that has them built) -- same
dependencies as the other scripts in this folder.
"""
import argparse
import os
import time

import numpy as np
import open3d as o3d
import rosbag
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument("bag", help="path to the rosbag (e.g. seq1.bag)")
parser.add_argument("--topic", default="/mocap_node/evk4/pose")
parser.add_argument("--ply", default=os.path.join(HERE, "..", "map", "fused_clean_mocap.ply"))
parser.add_argument("--extrinsic", default=os.path.join(HERE, "..", "calib", "T_body_camera.txt"))
parser.add_argument("--marker-radius", type=float, default=0.08)
parser.add_argument("--axis-size", type=float, default=0.3)
parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
parser.add_argument("--loop", action="store_true", help="loop playback when it reaches the end")
args = parser.parse_args()

T_body_camera = np.loadtxt(args.extrinsic, comments="#")
assert T_body_camera.shape == (4, 4), f"unexpected extrinsic shape {T_body_camera.shape}"


def load_poses(bag_path, topic):
    bag = rosbag.Bag(bag_path)
    ts, pos, quat = [], [], []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        p, q = msg.pose.position, msg.pose.orientation
        ts.append(msg.header.stamp.to_sec())
        pos.append([p.x, p.y, p.z])
        quat.append([q.x, q.y, q.z, q.w])
    bag.close()
    if not ts:
        raise RuntimeError(f"no messages on {topic} in {bag_path}")
    return np.array(ts), np.array(pos), np.array(quat)


def main():
    print(f"Loading {args.ply} ...")
    pcd = o3d.io.read_point_cloud(args.ply)
    print(f"  {len(pcd.points)} points")

    print(f"Loading poses from {args.bag} ({args.topic}) ...")
    ts, pos, quat = load_poses(args.bag, args.topic)
    rots = R.from_quat(quat)
    duration = ts[-1] - ts[0]
    print(f"  {len(ts)} poses, {duration:.1f}s")

    marker = o3d.geometry.TriangleMesh.create_sphere(radius=args.marker_radius)
    marker.paint_uniform_color([1.0, 0.1, 0.1])
    marker.compute_vertex_normals()
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.axis_size)

    vis = o3d.visualization.Visualizer()
    vis.create_window(f"Bag replay alignment check -- {os.path.basename(args.bag)}",
                       width=1600, height=1000)
    vis.add_geometry(pcd)
    vis.add_geometry(marker)
    vis.add_geometry(axis)
    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0.05, 0.05, 0.07])

    bbox = pcd.get_axis_aligned_bounding_box()
    ctr = vis.get_view_control()
    ctr.set_lookat(bbox.get_center())
    ctr.set_front([0, 0, -1])
    ctr.set_up([0, -1, 0])
    ctr.set_zoom(0.5)

    prev_T = np.eye(4)
    t_wall_start = time.time()
    i = 0
    print("Playing -- drag with the mouse to rotate/pan/zoom the view at any time.")

    try:
        while vis.poll_events():
            t_elapsed = (time.time() - t_wall_start) * args.speed
            t_target = ts[0] + t_elapsed
            if t_target > ts[-1]:
                if args.loop:
                    t_wall_start = time.time()
                    i = 0
                    continue
                else:
                    vis.update_renderer()
                    continue
            i = np.searchsorted(ts, t_target)
            i = min(i, len(ts) - 1)

            T_world_body = np.eye(4)
            T_world_body[:3, :3] = rots[i].as_matrix()
            T_world_body[:3, 3] = pos[i]
            T_world_cam = T_world_body @ T_body_camera

            delta = T_world_cam @ np.linalg.inv(prev_T)
            marker.transform(delta)
            axis.transform(delta)
            prev_T = T_world_cam

            vis.update_geometry(marker)
            vis.update_geometry(axis)
            vis.update_renderer()
    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()
        print("closed.")


if __name__ == "__main__":
    main()
