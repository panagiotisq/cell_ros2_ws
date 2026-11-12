#!/usr/bin/env python3
"""
Live alignment sanity check: shows the aligned room point cloud (map/fused_clean_mocap.ply)
in an Open3D window with a marker tracking the live /mocap_node/evk4/pose in real time.

Walk the rig around the room and watch whether the marker's position in the point cloud
matches where the rig physically is -- a live visual check of the COLMAP->mocap alignment.

Usage:
    source /opt/ros/noetic/setup.bash
    python3 live_pose_check.py [--topic /mocap_node/evk4/pose] [--ply ../map/fused_clean_mocap.ply]
"""
import argparse
import os
import threading

import numpy as np
import open3d as o3d
import rospy
from geometry_msgs.msg import PoseStamped

HERE = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument("--topic", default="/mocap_node/evk4/pose")
parser.add_argument("--ply", default=os.path.join(HERE, "..", "map", "fused_clean_mocap.ply"))
parser.add_argument("--extrinsic", default=os.path.join(HERE, "..", "calib", "T_body_camera.txt"))
parser.add_argument("--marker-radius", type=float, default=0.08)
parser.add_argument("--axis-size", type=float, default=0.3)
args = parser.parse_args()

T_body_camera = np.loadtxt(args.extrinsic, comments="#")
assert T_body_camera.shape == (4, 4), f"unexpected extrinsic shape {T_body_camera.shape}"

state = {"pose": None, "lock": threading.Lock()}


def pose_cb(msg: PoseStamped):
    p = msg.pose.position
    q = msg.pose.orientation
    with state["lock"]:
        state["pose"] = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    print(f"Loading {args.ply} ...")
    pcd = o3d.io.read_point_cloud(args.ply)
    print(f"  {len(pcd.points)} points")

    rospy.init_node("live_pose_check", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.topic, PoseStamped, pose_cb)
    print(f"Subscribed to {args.topic} -- waiting for the first pose...")

    # camera marker (red) -- what should align with the room reconstruction
    cam_marker = o3d.geometry.TriangleMesh.create_sphere(radius=args.marker_radius)
    cam_marker.paint_uniform_color([1.0, 0.1, 0.1])
    cam_marker.compute_vertex_normals()
    cam_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.axis_size)

    vis = o3d.visualization.Visualizer()
    vis.create_window("Live alignment check -- camera pose (via T_body_camera)",
                       width=1600, height=1000)
    vis.add_geometry(pcd)
    vis.add_geometry(cam_marker)
    vis.add_geometry(cam_axis)
    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0.05, 0.05, 0.07])

    # deliberate top-down bird's-eye view: looking straight down -Z with +Y as "up" on screen,
    # so left/right and forward/back on screen should directly match the room (assuming Z-up world).
    bbox = pcd.get_axis_aligned_bounding_box()
    ctr = vis.get_view_control()
    ctr.set_lookat(bbox.get_center())
    ctr.set_front([0, 0, -1])
    ctr.set_up([0, -1, 0])  # flipped 180 deg from before -- test if this matches your physical orientation
    ctr.set_zoom(0.5)

    prev_T_cam = np.eye(4)
    got_first = False

    try:
        while vis.poll_events():
            with state["lock"]:
                pose = state["pose"]
            if pose is not None:
                t, q = pose
                T_world_body = np.eye(4)
                T_world_body[:3, :3] = quat_to_R(q)
                T_world_body[:3, 3] = t

                T_world_camera = T_world_body @ T_body_camera

                delta_cam = T_world_camera @ np.linalg.inv(prev_T_cam)
                cam_marker.transform(delta_cam)
                cam_axis.transform(delta_cam)
                prev_T_cam = T_world_camera

                vis.update_geometry(cam_marker)
                vis.update_geometry(cam_axis)
                if not got_first:
                    print(f"First pose received: camera xyz={T_world_camera[:3, 3]}")
                    got_first = True
            vis.update_renderer()
    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()
        print("closed.")


if __name__ == "__main__":
    main()
