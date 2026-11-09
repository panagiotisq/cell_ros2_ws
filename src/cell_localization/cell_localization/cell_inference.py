"""Thin wrapper around CELL inference for deployment: depth render + model + confidence selection
+ PnP. NO edge refinement (excluded from the ROS pipeline by design).

The per-frame sequence mirrors CELL/eval_full.py exactly (get_corr / solve / floor0 selection),
but with a NO-GT depth path (eval_full uses push_fuse, which also builds the flow GT; deployment has
no GT, so we render the depth from the local map at the coarse pose guess and complete it with
partial_light). CELL's own modules do all the heavy lifting; nothing is re-derived here.

Imports from CELL are done lazily in __init__ so this module is importable for inspection without
torch / the compiled extensions present.
"""
from __future__ import annotations
import os
import sys
import numpy as np


class CellLocalizer:
    def __init__(self, cell_root: str, checkpoint: str, device: str,
                 crop, max_depth: float, iters: int = 24, pweight_seed: int = 0):
        if cell_root not in sys.path:
            sys.path.insert(0, cell_root)
            sys.path.insert(0, os.path.join(cell_root, "core"))
        import torch                                    # noqa: F401
        import torch.nn.functional as F
        import visibility, poselib                      # compiled extension + PnP solver
        from camera_model import CameraModel
        from utils_point import quat2mat
        from core.data_preprocess import Data_preprocess
        from core.depth_completion import sparse_to_dense
        from core.model_builder import build_model

        self.torch = torch
        self.F = F
        self.visibility = visibility
        self.poselib = poselib
        self.CameraModel = CameraModel
        self.quat2mat = quat2mat
        self.Data_preprocess = Data_preprocess
        self.sparse_to_dense = sparse_to_dense

        self.dev = torch.device(device)
        self.CH, self.CW, self.CX, self.CY = [int(v) for v in crop]
        self.MAXD = float(max_depth)
        self.iters = int(iters)
        self.rng = np.random.default_rng(pweight_seed)

        # generic checkpoint loader (infers the conf-head config from the saved weights)
        self.model = build_model("edge", True, checkpoint, self.dev, [0])

        # solver configs (copied verbatim from eval_full.py)
        self.RANSAC = {"max_reproj_error": 12.0, "seed": 0, "progressive_sampling": False,
                       "max_prosac_iterations": 50000, "real_focal_check": False}
        self.BUNDLE = {'loss_type': "HUBER", 'loss_scale': 1.0, 'gradient_tol': 1e-8,
                       'step_tol': 1e-8, 'initial_lambda': 1e-3, 'min_lambda': 1e-10,
                       'max_lambda': 1e10, 'verbose': False}

    # ---- geometry helpers (ported from eval_full.py) -------------------------
    def _get_corr(self, flow_up, depth, calib):
        torch, vis, np_ = self.torch, self.visibility, np
        out = torch.zeros(flow_up.shape).to(self.dev)
        pdi = torch.zeros(depth.shape).to(self.dev) + 1000.
        out = vis.image_warp_index(depth.to(self.dev), flow_up.int(), pdi, out,
                                   depth.shape[3], depth.shape[2])
        pdi[pdi == 1000.] = 0.
        uv = out.cpu().permute(0, 2, 3, 1).numpy()
        di = (depth.cpu().numpy() * self.MAXD)[0, 0] * ((uv[0, :, :, 0] != 0) + (uv[0, :, :, 1] != 0))
        cam = self.CameraModel()
        cp = calib[0].clone().cpu().numpy()
        cp[2] += self.CW / 2. - (self.CY + self.CY + self.CW) / 2.   # == -CY : principal point for the crop
        cp[3] += self.CH / 2. - (self.CX + self.CX + self.CH) / 2.   # == -CX
        cam.focal_length = cp[:2]
        cam.principal_point = cp[2:]
        p3, p2, idx = cam.deproject_pytorch(di, uv[0, :, :, :])
        return p2, p3, idx, cp

    def _solve(self, p2, p3, cp):
        torch = self.torch
        if len(p3) < 4:
            return None
        cam = {'model': 'SIMPLE_PINHOLE', 'width': self.CW, 'height': self.CH,
               'params': [cp[0], cp[2], cp[3]]}
        try:
            pose, _ = self.poselib.estimate_absolute_pose(p2.copy(), p3.copy(), cam,
                                                          self.RANSAC, self.BUNDLE)
        except TypeError:
            im, _ = self.poselib.estimate_absolute_pose(p2.copy(), p3.copy(), cam, self.RANSAC)
            pose = im.pose
        R = torch.tensor(pose.q); Tt = torch.tensor(pose.t)
        R[1:] *= -1; Tt *= -1
        return R, Tt

    def _sel_floor0(self, conf):
        # floor0: keep each correspondence w.p. conf_norm (per-frame min-max). == sel_pweight(floor=0).
        c = conf.astype(float); lo = c.min(); hi = c.max()
        cn = (c - lo) / (hi - lo) if hi > lo else np.ones_like(c)
        return np.where(self.rng.random(len(c)) < cn)[0]

    # ---- the deployment forward ----------------------------------------------
    def localize(self, event_frame, local_map_pc, pose_guess_R, pose_guess_T, calib):
        """
        event_frame   : (H_full, W_full, 2) float32 time-surface from EventRepresentation.build_frame
        local_map_pc  : (N, 4) float32 point cloud (map cropped around the guess), CELL convention
        pose_guess_R  : (4,) quaternion  }  the coarse guess as a perturbation RT fed to push_input
        pose_guess_T  : (3,) translation }  (same role as test_RT in eval) -- TODO: confirm convention
        calib         : (4,) [fx, fy, cx, cy] at the representation resolution
        returns (quat (4,), t (3,)) absolute camera pose (Flow2Pose convention) or None.
        """
        torch, F = self.torch, self.F
        ev = torch.from_numpy(event_frame).permute(2, 0, 1).float()          # (2, H, W)
        pc = torch.from_numpy(local_map_pc).float()
        Re = torch.from_numpy(np.asarray(pose_guess_R, dtype=np.float32))
        Te = torch.from_numpy(np.asarray(pose_guess_T, dtype=np.float32))
        calib_t = torch.from_numpy(np.asarray(calib, dtype=np.float32))

        dp = self.Data_preprocess(calib_t.unsqueeze(0), 3, 5)
        # render the SPARSE depth at the guessed pose (no GT needed): push_input returns normalized
        # sparse depth (/MAXD), center-cropped to (CH, CW), plus the cropped event frame.
        ei, di_sparse, _, _ = dp.push_input(ev.unsqueeze(0), [pc], Te.unsqueeze(0), Re.unsqueeze(0),
                                            self.dev, MAX_DEPTH=self.MAXD, h=self.CH, w=self.CW,
                                            split="test")
        # partial_light completion (as in training): complete in METRES then renormalize.
        sparse_m = (di_sparse[0, 0].cpu().numpy() * self.MAXD)
        dense_m = self.sparse_to_dense(sparse_m, fill_level="partial_light")
        di = torch.from_numpy(dense_m / self.MAXD).float().to(self.dev).unsqueeze(0).unsqueeze(0)
        # NOTE: training completes BEFORE the crop (push_fuse); here we complete AFTER (push_input
        # crops internally). partial_light is a 1-px-local op, so they differ only at the crop border.
        # TODO: if exact parity matters, render+complete full-res then crop.

        with torch.no_grad():
            _, flow_up, _, cf = self.model(di, ei, iters=self.iters, test_mode=True,
                                           output_conf=True, idx=0)
        cm = F.softplus(cf)[0, 0].cpu().numpy()
        p2, p3, idx, cp = self._get_corr(flow_up, di, calib_t.unsqueeze(0))
        if p3.shape[0] < 20:
            return None
        r = idx[:, 0].astype(int); c = idx[:, 1].astype(int)
        conf = cm[r, c]
        sel = self._sel_floor0(conf)
        if len(sel) < 8:
            return None
        out = self._solve(p2[sel], p3[sel], cp)
        if out is None:
            return None
        R, Tt = out
        return np.asarray(R).ravel(), np.asarray(Tt).ravel()
