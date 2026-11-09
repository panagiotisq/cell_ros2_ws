"""CELL relocalization node (ROS 2 / rclpy).

Pipeline per localization (triggered by each incoming coarse pose guess):
    rolling event buffer (last ~100 ms) -> STC/Trail frame (ei)
    + prior LiDAR map cropped at the guess -> depth (di)
    -> CELL model -> flow + confidence -> floor0 selection -> PnP
    -> publish estimated absolute pose (PoseStamped) [+ TF map->camera].
Edge refinement is intentionally excluded.

Two parts are stubbed pending info from the lab (see TODOs):
  (1) the event message type / decode  -> depends on the sensor + driver
  (2) the STC/Trail representation backend (SDK vs SDK-free) -> see event_representation.py
Also stubbed: the prior-map load + local crop, and the pose-guess / output frame conventions.
"""
import collections
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo

from .event_representation import EventRepresentation
from .cell_inference import CellLocalizer


class CellLocalizationNode(Node):
    def __init__(self):
        super().__init__('cell_localization')
        gp = self._declare_and_get

        # --- params ---
        self.cell_root = gp('cell_root', '/home/panosk/CELL')
        self.checkpoint = gp('checkpoint', '')
        self.device = gp('device', 'cuda:0')
        self.scene = gp('scene', '')
        self.crop = list(gp('crop', [288, 512, 36, 64]))
        self.max_depth = float(gp('max_depth', 10.0))
        self.fx = float(gp('fx', 0.0)); self.fy = float(gp('fy', 0.0))
        self.cx = float(gp('cx', 0.0)); self.cy = float(gp('cy', 0.0))
        self.iters = int(gp('iters', 24))
        self.pweight_seed = int(gp('pweight_seed', 0))
        self.event_window_us = int(gp('event_window_us', 100000))
        self.half_resolution = bool(gp('half_resolution', True))
        self.representation_backend = gp('representation_backend', 'sdk_free')
        self.map_file = gp('map_file', '')
        self.local_map_radius_m = float(gp('local_map_radius_m', 30.0))
        self.map_frame = gp('map_frame', 'map')
        self.camera_frame = gp('camera_frame', 'camera')
        self.publish_tf = bool(gp('publish_tf', True))
        self.events_topic = gp('events_topic', '/event_camera/events')
        self.pose_guess_topic = gp('pose_guess_topic', '/pose_guess')
        self.camera_info_topic = gp('camera_info_topic', '/event_camera/camera_info')
        self.pose_out_topic = gp('pose_out_topic', '/cell/pose')

        # --- geometry from CELL scene_config if a scene is given ---
        if self.scene:
            import sys, os
            if self.cell_root not in sys.path:
                sys.path.insert(0, self.cell_root); sys.path.insert(0, os.path.join(self.cell_root, 'core'))
            from core.scene_config import scene_geometry
            g = scene_geometry(self.scene, 'm3ed')
            self.crop = [int(v) for v in g['crop']]
            self.max_depth = float(g['max_depth'])
            self.get_logger().info(f"scene_config[{self.scene}]: crop={self.crop} max_depth={self.max_depth}")

        # --- CELL inference wrapper (loads the model) ---
        self.get_logger().info("loading CELL model ...")
        self.localizer = CellLocalizer(self.cell_root, self.checkpoint, self.device,
                                       self.crop, self.max_depth, self.iters, self.pweight_seed)

        # --- event representation builder ---
        # rows/cols = frame size = full-sensor // k. TODO(sensor): set full-sensor res from the driver.
        full_rows, full_cols = 720, 1280                 # M3ED Prophesee default; fix per sensor
        k = 2 if self.half_resolution else 1
        self.repr = EventRepresentation(full_rows // k, full_cols // k,
                                        time_window_us=self.event_window_us,
                                        half_resolution=self.half_resolution,
                                        backend=self.representation_backend)

        # --- state ---
        # rolling event buffer of (x, y, t_us, p); a deque trimmed by timestamp on each insert
        self._ev = collections.deque()
        self._calib = None          # [fx, fy, cx, cy]; from CameraInfo or params
        self._map = None            # (N,4) prior cloud; loaded lazily
        if any(v > 0 for v in (self.fx, self.fy)):
            self._calib = np.array([self.fx, self.fy, self.cx, self.cy], dtype=np.float32)

        # --- pubs / subs ---
        self.pub = self.create_publisher(PoseStamped, self.pose_out_topic, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(PoseStamped, self.pose_guess_topic, self._on_pose_guess, 10)
        self._subscribe_events()
        if self.publish_tf:
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)
        self.get_logger().info("CELL localizer ready (waiting for events + pose guesses).")

    def _declare_and_get(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ---- subscriptions -------------------------------------------------------
    def _subscribe_events(self):
        # TODO(sensor): swap in the real event message type once known and decode to (x, y, t_us, p).
        #   Prophesee / Metavision ROS 2:  event_camera_msgs/EventPacket
        #   DVS / DAVIS:                   dvs_msgs/EventArray
        # Until then this is a placeholder so the node builds; it logs once and does nothing.
        self.get_logger().warn(
            "events subscription is a STUB (sensor/message type TBD). Fill in _subscribe_events + "
            "_on_events once the sensor is known.")
        # Example once decided (dvs_msgs):
        # from dvs_msgs.msg import EventArray
        # self.create_subscription(EventArray, self.events_topic, self._on_events, 10)

    def _on_events(self, msg):
        # TODO(sensor): append decoded events to self._ev, then trim to the time window.
        # for e in msg.events:
        #     t_us = int(e.ts.sec * 1_000_000 + e.ts.nanosec // 1000)
        #     self._ev.append((e.x, e.y, t_us, 1 if e.polarity else 0))
        # self._trim_buffer(latest_t_us)
        raise NotImplementedError

    def _trim_buffer(self, now_us):
        lo = now_us - self.event_window_us
        while self._ev and self._ev[0][2] < lo:
            self._ev.popleft()

    def _on_camera_info(self, msg: CameraInfo):
        if self._calib is None:
            k = msg.k  # row-major 3x3
            self._calib = np.array([k[0], k[4], k[2], k[5]], dtype=np.float32)  # fx, fy, cx, cy
            self.get_logger().info(f"got intrinsics from CameraInfo: {self._calib.tolist()}")

    # ---- the trigger: a coarse pose guess arrives ----------------------------
    def _on_pose_guess(self, msg: PoseStamped):
        if self._calib is None:
            self.get_logger().warn("no intrinsics yet (CameraInfo/params); skipping guess.")
            return
        if len(self._ev) == 0:
            self.get_logger().warn("event buffer empty; skipping guess.")
            return

        # 1) build the STC/Trail frame from the current buffer
        ev = np.array(self._ev, dtype=np.int64)
        frame = self.repr.build_frame(ev[:, 0], ev[:, 1], ev[:, 2], ev[:, 3])

        # 2) prior map cropped around the guess  -- TODO: real load + crop (generate_local_maps.py)
        local_map = self._get_local_map(msg)
        if local_map is None:
            self.get_logger().warn("no local map; skipping guess.")
            return

        # 3) pose guess -> (R_quat, T)  -- TODO: confirm the map->guess convention expected by push_input
        Rq, T = self._pose_to_RT(msg)

        # 4) run CELL
        result = self.localizer.localize(frame, local_map, Rq, T, self._calib)
        if result is None:
            self.get_logger().warn("localization returned None (too few correspondences).")
            return
        quat, t = result

        # 5) publish  -- TODO: map CELL's Flow2Pose quaternion convention to ROS (map->camera)
        self._publish_pose(quat, t, msg.header.stamp)

    # ---- stubs to finalize with deployment specifics -------------------------
    def _get_local_map(self, pose_guess: PoseStamped):
        # TODO: load self.map_file once (e.g. open3d/pcl), then crop a local_map_radius_m submap
        # around the guess and return it as (N,4) in CELL convention (match generate_local_maps.py).
        return None

    def _pose_to_RT(self, pose_guess: PoseStamped):
        q = pose_guess.pose.orientation
        p = pose_guess.pose.position
        Rq = np.array([q.w, q.x, q.y, q.z], dtype=np.float32)   # mathutils.Quaternion order (w,x,y,z)
        T = np.array([p.x, p.y, p.z], dtype=np.float32)
        return Rq, T

    def _publish_pose(self, quat, t, stamp):
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.map_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = [float(v) for v in t[:3]]
        # Flow2Pose quat is (w,x,y,z); ROS wants (x,y,z,w). TODO: verify the frame convention too.
        out.pose.orientation.w = float(quat[0])
        out.pose.orientation.x = float(quat[1])
        out.pose.orientation.y = float(quat[2])
        out.pose.orientation.z = float(quat[3])
        self.pub.publish(out)
        if self.publish_tf:
            from geometry_msgs.msg import TransformStamped
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.camera_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
                [float(v) for v in t[:3]]
            tf.transform.rotation.w = float(quat[0]); tf.transform.rotation.x = float(quat[1])
            tf.transform.rotation.y = float(quat[2]); tf.transform.rotation.z = float(quat[3])
            self._tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = CellLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
