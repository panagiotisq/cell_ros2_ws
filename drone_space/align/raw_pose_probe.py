#!/usr/bin/env python3
"""Print raw position + axis-angle rotation vector from /mocap_node/evk4/pose, at ~3Hz.
No visualization, no viewpoint ambiguity -- just numbers, for diagnosing axis conventions.
"""
import os
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
T_body_camera = np.loadtxt(os.path.join(HERE, "..", "calib", "T_body_camera.txt"), comments="#")

last = {"t": 0.0}
ref = {"body": None, "cam": None}


def cb(msg: PoseStamped):
    now = time.time()
    if now - last["t"] < 0.33:
        return
    last["t"] = now
    p = msg.pose.position
    q = msg.pose.orientation
    T_world_body = np.eye(4)
    T_world_body[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T_world_body[:3, 3] = [p.x, p.y, p.z]
    T_world_cam = T_world_body @ T_body_camera

    if ref["body"] is None:
        ref["body"] = T_world_body
        ref["cam"] = T_world_cam
        print("Reference pose captured. Now do ONE clean motion and watch which component moves.")
        return

    dp_body = T_world_body[:3, 3] - ref["body"][:3, 3]
    dp_cam = T_world_cam[:3, 3] - ref["cam"][:3, 3]
    dR_body = R.from_matrix(T_world_body[:3, :3] @ ref["body"][:3, :3].T)
    dR_cam = R.from_matrix(T_world_cam[:3, :3] @ ref["cam"][:3, :3].T)
    rotvec_body = dR_body.as_rotvec(degrees=True)
    rotvec_cam = dR_cam.as_rotvec(degrees=True)

    if np.max(np.abs(rotvec_body)) < 3.0 and np.max(np.abs(dp_body)) < 0.10:
        return  # ignore noise -- only report real, deliberate motion
    print(f"BODY d_pos: x={dp_body[0]:+7.3f} y={dp_body[1]:+7.3f} z={dp_body[2]:+7.3f}   |   "
          f"d_rotvec(deg): x={rotvec_body[0]:+7.2f} y={rotvec_body[1]:+7.2f} z={rotvec_body[2]:+7.2f}")
    print(f"CAM  d_pos: x={dp_cam[0]:+7.3f} y={dp_cam[1]:+7.3f} z={dp_cam[2]:+7.3f}   |   "
          f"d_rotvec(deg): x={rotvec_cam[0]:+7.2f} y={rotvec_cam[1]:+7.2f} z={rotvec_cam[2]:+7.2f}")


rospy.init_node("raw_pose_probe", anonymous=True)
rospy.Subscriber("/mocap_node/evk4/pose", PoseStamped, cb)
print("Printing pose delta at ~3Hz, relative to the first pose received. Ctrl+C to stop.")
print("d_rotvec direction = rotation axis, magnitude = angle (deg) rotated since the start.")
rospy.spin()
