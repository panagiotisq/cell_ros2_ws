"""Build the STC/Trail 2-channel time-surface frame (the model input `ei`) from a buffer of raw
events, ONLINE.

This is the one genuinely new piece of engineering the ROS deployment adds. In the paper/eval
pipeline the raw-events -> frame step was done OFFLINE by CELL/tools/m3ed/event2frame.py using the
proprietary Metavision SDK; the model was then fed pre-rendered .npy frames. A rosbag contains raw
events, so the node must produce the representation here.

Two backends (pick via the `representation_backend` param; decide with the lab PC):
  - "sdk":      reuse Metavision SpatioTemporalContrast + Trail filters (bit-identical to the paper,
                needs a licensed Metavision install on the deployment PC).
  - "sdk_free": the numba STC/Trail reimplementation (portable, no license; slight numerical diff).

Both must return the SAME tensor shape/semantics the model was trained on:
  float32 array (H, W, 2) == (rows, cols, [pos_polarity_surface, neg_polarity_surface]),
  time-surface values with the window start subtracted and negatives clipped to 0
  (see event2frame.py, method "ours_denoise_stc_trail").
"""
from __future__ import annotations
import numpy as np


class EventRepresentation:
    def __init__(self, rows: int, cols: int, time_window_us: int = 100_000,
                 half_resolution: bool = True, backend: str = "sdk_free"):
        # rows/cols are the FULL sensor resolution // k (k=2 if half_resolution), i.e. the frame size.
        self.rows = rows
        self.cols = cols
        self.k = 2 if half_resolution else 1
        self.time_window_us = time_window_us
        self.backend = backend
        if backend == "sdk":
            self._init_sdk()
        elif backend == "sdk_free":
            self._init_sdk_free()
        else:
            raise ValueError(f"unknown representation backend {backend!r}")

    # ---- SDK backend ---------------------------------------------------------
    def _init_sdk(self):
        # Mirror event2frame.py lines 96-141. Full-sensor filter dims are 1280x720 for the M3ED
        # Prophesee; set from the real sensor once known.
        # from metavision_sdk_cv import (ActivityNoiseFilterAlgorithm,
        #                                 TrailFilterAlgorithm, SpatioTemporalContrastAlgorithm)
        # self._trail = TrailFilterAlgorithm(1280, 720, self.time_window_us // 10)
        # self._stc   = SpatioTemporalContrastAlgorithm(1280, 720, self.time_window_us // 10, True)
        raise NotImplementedError(
            "SDK backend: port event2frame.py's STC+Trail block here once the Metavision SDK and the "
            "sensor resolution are confirmed.")

    def _init_sdk_free(self):
        # TODO: bring over the numba STC + latest-timestamp-surface reimplementation (earlier notes:
        # 'stc_trail is a latest-ts time-surface'; reproduced SDK-free via numba STC + maximum.at).
        pass

    # ---- public API ----------------------------------------------------------
    def build_frame(self, x: np.ndarray, y: np.ndarray, t: np.ndarray, p: np.ndarray) -> np.ndarray:
        """events (already decoded from the ROS message) -> (H, W, 2) float32 time-surface.

        x, y : int pixel coords at FULL sensor resolution (divided by self.k inside).
        t    : int64 microseconds.
        p    : polarity (>0 positive, else negative).
        Only the last `time_window_us` of events should be passed in (the node keeps the buffer).
        """
        if self.backend == "sdk":
            return self._build_sdk(x, y, t, p)
        return self._build_sdk_free(x, y, t, p)

    def _build_sdk(self, x, y, t, p):
        raise NotImplementedError("see _init_sdk TODO")

    def _build_sdk_free(self, x, y, t, p):
        # Placeholder faithful to the OUTPUT CONTRACT so the rest of the pipeline can be wired/tested
        # with synthetic data. Replace with the real numba STC/Trail (lab reimplementation).
        # TODO(representation): real STC + Trail filtering; this stub is a plain 2-channel latest-ts
        # surface with no denoising — correct shape/semantics, NOT the trained representation.
        img = np.zeros((self.rows, self.cols, 2), dtype=np.float32)
        if len(t) == 0:
            return img
        yy = (y // self.k).astype(np.int64)
        xx = (x // self.k).astype(np.int64)
        ok = (yy >= 0) & (yy < self.rows) & (xx >= 0) & (xx < self.cols)
        yy, xx, tt, pp = yy[ok], xx[ok], t[ok], p[ok]
        ch = np.where(pp > 0, 0, 1)
        # latest timestamp wins per (pixel, polarity)
        np.maximum.at(img, (yy, xx, ch), tt.astype(np.float32))
        pos = img > 0
        img[pos] -= t.min()
        img[img < 0] = 0
        return img
