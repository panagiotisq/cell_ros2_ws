# On-site room reconstruction (COLMAP) — PC3 steps

**Audience:** a fresh Claude Code session running on the drone-space ROS1 PC ("PC3"), plus the user
driving it. **Scope:** produce a **dense 3D point cloud of the room** from a phone video, on-site, so
the capture can be re-shot immediately if it's poor. This cloud becomes CELL's prior map
(see `DATASET_PLAN.md`).

**Ground rules**
- Do **NOT** modify or run the lab's existing ROS1 acquisition project (the bag-recording scripts).
  Work only inside the user's own folder, created *beside* it.
- No system-wide installs / no `sudo` unless the user approves. Use a conda env in user space.

---

## For the user — one-time setup (before pointing Claude at this file)

1. Clone this repo into your own folder, parallel to the existing acquisition project:
   ```bash
   cd ~/workspaces/ros            # keep it BESIDE the existing project, not inside it
   git clone https://github.com/panagiotisq/cell_ros2_ws.git
   mkdir -p ~/workspaces/ros/panos_recon/{video,frames,colmap}
   ```
2. Install Claude Code if absent (user-space Node via nvm — no sudo). Copy-paste the whole block;
   the `export`/source lines are what make `node` appear in the current shell (the #1 gotcha):
   ```bash
   # a) install nvm
   curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash

   # b) load nvm into THIS shell (installer does NOT do this for the current session)
   export NVM_DIR="$HOME/.nvm"
   [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"

   # c) install Node 20 and verify
   nvm install 20
   node --version        # expect v20.x
   npm --version

   # d) install Claude Code
   npm install -g @anthropic-ai/claude-code
   claude --version
   ```
   Troubleshooting:
   - `nvm: command not found` after (a) → you skipped (b); run the two `export`/source lines, or open
     a new terminal.
   - `curl: command not found` → `sudo apt install curl` (or use `wget`); if no sudo, ask.
   - `node` still missing after (c) → run `nvm use 20`, then re-check `node --version`.
3. Open Claude one level **above** your files, and give it this doc:
   ```bash
   cd ~/workspaces/ros
   claude
   ```
   First message to Claude:
   > "Read cell_ros2_ws/drone_space/PC3_RECON_STEPS.md and cell_ros2_ws/drone_space/DATASET_PLAN.md,
   >  then follow PC3_RECON_STEPS.md. We're on the drone-space ROS1 PC. Do the environment checks and
   >  COLMAP setup, then STOP and ask me for the room video. Don't touch the existing acquisition project."

---

## For Claude — execute in order

### Step 1 — Environment check (report back before continuing)
```bash
nvidia-smi            # CUDA GPU present? note name + driver (dense MVS needs it)
lsb_release -a        # Ubuntu version
df -h ~               # free disk (dense can need several GB)
which conda || echo "no conda"
```
**Decision:** GPU present → full sparse+dense on-site. No GPU → sparse only on-site (fast, verifies
the capture), leave dense for the lab PC GPU. Tell the user which path applies.

### Step 2 — Install COLMAP (user-space conda)
```bash
conda create -n recon -c conda-forge colmap ffmpeg -y
conda activate recon
colmap -h >/dev/null && echo "colmap OK"
```
If conda is absent and the user grants sudo on Ubuntu 20.04+: `sudo apt install colmap ffmpeg`.
Otherwise ask the user (an AppImage/Docker fallback exists).

> **COLMAP version note:** conda-forge currently ships **COLMAP 4.2.1**, which **renamed** some options
> relative to the COLMAP 3.x docs/examples found online. Use `--FeatureExtraction.use_gpu` (NOT the old
> `--SiftExtraction.use_gpu`); GPU is on by default. The `automatic_reconstructor` command in Step 5 is
> unchanged and works as written. If any step-by-step COLMAP command errors with *"unrecognised
> option"*, this rename is why — check `colmap <command> -h` for the current flag name.

### Step 3 — STOP and ask the user for the room video
> "Environment ready (GPU: yes/no). Please shoot a phone video of the room — slow, steady, full
>  coverage, 70–80% overlap, multiple heights/angles, even light, no motion blur, ~1–3 min — and put
>  it at `~/workspaces/ros/panos_recon/video/room.mp4` (or tell me the path)."

### Step 4 — Downsample the video to frames
```bash
ffmpeg -i ~/workspaces/ros/panos_recon/video/room.mp4 -vf fps=3 \
       ~/workspaces/ros/panos_recon/frames/%04d.jpg
ls ~/workspaces/ros/panos_recon/frames | wc -l   # aim ~150–400 frames
```
Adjust `fps` if too few/many (coverage matters more than raw count). Drop obviously blurry frames.

### Step 5 — Reconstruct
GPU path (sparse + dense):
```bash
colmap automatic_reconstructor \
  --image_path ~/workspaces/ros/panos_recon/frames \
  --workspace_path ~/workspaces/ros/panos_recon/colmap \
  --dense 1
# dense cloud -> ~/workspaces/ros/panos_recon/colmap/dense/0/fused.ply
```
No-GPU path: same command with `--dense 0` (sparse only; run dense later on the lab PC). If
`automatic_reconstructor` reports no CUDA, fall back to `--dense 0` and note it.

### Step 6 — Inspect + report
From the COLMAP log, report: how many images registered, whether the room is covered, the output
path. If coverage is poor (few images registered, big holes), **advise a re-shoot with more overlap**
— catching this on-site is the whole reason to reconstruct here.

### Step 7 — What to hand off to the lab PC
- `fused.ply` (dense) — or the sparse model folder if no GPU.
- Keep the `frames/` and the COLMAP `colmap/` workspace (lets dense be re-run on the lab PC).
- **Note:** scale is arbitrary (photo-only SfM). Metric scale + alignment to the **mocap world** happen
  on the lab PC using measured control points — see `DATASET_PLAN.md`. While on-site, measure **≥4–6
  well-spread control points** (room corners, distinct fixed objects) in mocap-world coordinates; the
  alignment needs them.
