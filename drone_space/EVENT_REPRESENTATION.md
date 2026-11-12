# Event representation — spec for full-sequence generation

Decided on-site after comparing against the real Metavision SDK STC/Trail filters on matched
samples: the SDK filters only reduced background noise speckle slightly, not worth ~10-20x more
processing time at full-sequence scale. **Use the representation below — raw/unfiltered events,
no Metavision SDK involved.**

## Representation: single-channel exponential-decay time-surface

For each 100 ms window `[w_start, w_end)`:
- For every pixel `(x, y)`: find the **most recent raw CD event at that pixel within the window**
  (polarity is ignored — both ON and OFF events count equally; only recency matters).
- If no event occurred at that pixel in the window: value = **0**.
- Otherwise: `value = exp(-age / TAU) * 255`, where `age = w_end - t_last_event` (seconds),
  clipped to `[0, 255]` and cast to `uint8`.

**Parameters:**
- `WINDOW_S = 0.100` (100 ms window per frame, ~10 Hz non-overlapping cadence for a continuous
  sequence of frames)
- `TAU = 0.030` (30 ms decay constant — tuned so the most recent events render near-white and the
  frame fades to near-black by ~90-100 ms; this is deliberately faster than a linear 0-100ms ramp)
- Output: single-channel `uint8`, 1280×720 (native EVK4 resolution)

## Input

- Topic: `/prophesee/camera/cd_events_buffer`, type `prophesee_event_msgs/EventArray`
  (`events[]`, each with `x`, `y`, `ts`, `polarity`)
- Ground-truth pose: `/mocap_node/evk4/pose`, type `geometry_msgs/PoseStamped`, paired with
  `calib/T_body_camera.txt` (confirmed convention: camera→body,
  `T_world_camera = T_world_body · T_body_camera`) and the aligned map `map/fused_clean_mocap.ply`.

## Critical gotcha: rosbag read-padding

When pulling events for a window via `bag.read_messages(topic, start_time=..., end_time=...)`,
**pad the read range by an extra ~50 ms on each side** (`READ_PAD = 0.050`) beyond the window's own
boundaries. Rosbag's message-level `start_time`/`end_time` filtering is based on the message's
*recording* timestamp, which lags behind the actual event timestamps inside it (publish latency).
Without this padding, the most recent slice of events near the window's end gets silently dropped —
this was confirmed empirically (events up to ~15ms before the window end were missing without the
pad). The per-event window membership test (`w_start <= t < w_end`) still applies using each event's
own precise timestamp, after the padded read.

## Performance: vectorize, don't loop per-event

A naive per-event Python loop (e.g. `count[ev.y, ev.x] += 1` inside a `for ev in events` loop) is
**far too slow** at real sequence scale (tens to hundreds of millions of raw events per sequence) —
confirmed empirically: one attempt took 900+s without finishing a single 7s clip. Instead:

1. Read raw events in chunks (e.g. 1M events) into numpy structured arrays
   (`dtype=[('x','<u2'),('y','<u2'),('p','<i2'),('t','<i8')]`, `t` in integer microseconds or float
   seconds — pick one and stay consistent).
2. Concatenate chunks for the whole sequence, sort once by `t`.
3. Compute bin edges for all windows up front, use `np.searchsorted` on the sorted timestamp array
   to get each window's start/end slice directly — no per-event binning loop.
4. Per window: sort that slice by `t` ascending, compute `age`/`decay` vectorized, then scatter via
   `img[y, x] = decay` — because duplicate-index assignment in numpy keeps the *last* write, sorting
   ascending means the most recent event at each pixel automatically wins, for free.

With this approach, frame-building adds only ~20-25s on top of raw extraction per 7s clip (raw
extraction itself is the dominant, essentially irreducible cost — ROS message deserialization in
Python measured at ~0.28M events/s).
