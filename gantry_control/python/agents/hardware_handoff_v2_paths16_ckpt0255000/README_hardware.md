# Hardware Handoff: v2 paths16 policy (path-blind)

This package is a compact deployment handoff for the v2 path-blind SAC policy trained in:
- `rl_sac_v2_pathblind_runs/paths16_backup_2`

Selected checkpoint in this handoff:
- `checkpoint_0255000.pt`
- actor-only copy: `actor_only_0255000.pt`

Important note:
- This is a multi-path, path-blind controller. It does not use any path geometry at inference time.
- For hardware testing, only the observation/kinematics interface matters.

## Package contents

- `checkpoint_0255000.pt`: original full SAC checkpoint from the chosen run
- `actor_only_0255000.pt`: actor weights only, for easier deployment
- `source_config.json`: config snapshot saved with the source run
- `checkpoint_info.json`: provenance and selection notes
- `io_contract.json`: shapes, field order, timing, and action semantics
- `example_input_shapes.npz`: shape-only example of `sensor_history` and `kin`
- `policy_model.py`: standalone actor definition compatible with the actor weights
- `deploy_v2_paths16.py`: minimal inference wrapper for hardware integration
- `simulate_v2_paths16.py`: deterministic simulator rollout entry point using the handoff package

## What the model expects

Observation is split into two parts.

1. `sensor_history`
- shape: `(16, 3, 3, 2)`
- meaning: 16 recent sensor frames from the 3x3 whisker array
- timing: one sensor frame every `12.5 ms` (`80 Hz`)

2. `kin`
- shape: `(6,)`
- exact order:
  - `x_mm`
  - `y_mm`
  - `vx_mm_per_ms`
  - `vy_mm_per_ms`
  - `prev_y_velocity_mm_per_ms`
  - `normalized_time`

Important semantic note:
- `prev_action` in training was the previous **physical global y velocity** after scaling, in `mm/ms`.

## Whisker ordering

The tensor row order matches whisker-id order from training:
- row 0: `w001 w002 w003`
- row 1: `w004 w005 w006`
- row 2: `w007 w008 w009`

This corresponds to the training tensor layout and must be preserved on hardware.

## Control timing

The trained policy was used at `20 Hz`:
- sensor update every `12.5 ms`
- one control action every `4` sensor frames
- so one control step = `50 ms`

Recommended hardware-side schedule:
- push one new whisker frame every `12.5 ms`
- run the policy every 4 pushed frames
- hold the resulting command for the next 4 sensor frames

## Action meaning

The policy output is one scalar in `[-1, 1]`.
It is interpreted as a **normalized global-y velocity command**.

Conversion used during training:
- `y_velocity = action * 0.15 mm/ms`
- `x_velocity` is fixed at `0.15 mm/ms`
- final velocity command:
  - `velocity_xy = [0.15, y_velocity]`

If your hardware API expects the same command convention as the simulator, convert that vector to:
- `speed_ratio = ||velocity_xy|| / 0.4`
- `direction_radian = atan2(vy, vx)`

If your hardware accepts direct `vx, vy` commands instead, you can use `command_vx_mm_per_ms` and `command_vy_mm_per_ms` from the deployment wrapper directly.

## Using the hardware deployment wrapper

From this folder:

```bash
python deploy_v2_paths16.py --package-dir . --device cpu
```

In code, the intended usage is:
1. create `V2Paths16HardwareRunner`
2. call `reset_episode()` at the start of a hardware trial
3. every sensor frame, call `push_sensor_frame(frame_3x3x2)`
4. every 4 frames, call `compute_command(x, y, vx, vy, time_ms)`

What this means operationally:
- sensor frames arrive at `80 Hz`
- the runner keeps a rolling 16-frame buffer, which corresponds to the latest `200 ms`
- the actor is queried only every 4 frames, so the control rate is `20 Hz`
- consecutive policy inputs overlap heavily:
  - first action uses frames `1..16`
  - next action uses frames `5..20`
  - next action uses frames `9..24`
- when `compute_command(...)` returns `None`, the caller should keep holding the previous command
- when it returns a command dict, that new command should be held for the next 4 sensor frames

Minimal integration sketch:

```python
runner = V2Paths16HardwareRunner(package_dir=".")
runner.reset_episode()
last_command = None

while hardware_trial_is_running:
    frame = read_whisker_frame()          # shape (3, 3, 2)
    x, y, vx, vy, t_ms = read_pose_state()
    runner.push_sensor_frame(frame)

    command = runner.compute_command(x, y, vx, vy, t_ms)
    if command is not None:
        last_command = command
        send_hardware_command(command)
    else:
        hold_previous_command(last_command)
```

The returned dict contains:
- normalized action
- physical y velocity
- global `vx, vy`
- `speed_ratio`
- `direction_radian`

## Running one simulator rollout

This package also includes a small simulator-side helper:

```bash
python simulate_v2_paths16.py --package-dir . --device cpu
```

By default it uses `actor_only_0255000.pt`, rebuilds the simulator config from `source_config.json`, runs a deterministic rollout, and writes results to a folder such as `sim_eval_actor_only_0255000/`.

Useful options:
- `--weights checkpoint_0255000.pt` to test the full checkpoint instead of the actor-only export
- `--path-sub path1` to force a specific path
- `--xloc-start ... --yloc-start ...` to override the start position
- `--output-dir ...` to choose where plots and CSV files are written

## Safety / deployment recommendations

Before live hardware control:
- verify whisker channel ordering carefully
- verify sensor units match training assumptions
- verify `x, y, vx, vy` are available online and in the same units
- test in shadow mode first: compute actions but do not actuate
- keep command saturation on the hardware side as an extra safety layer

## Scope of this package

This handoff package is for **inference only**.
It does not contain:
- replay buffer
- critics
- SAC training loop
- evaluation job scripts

Those are unnecessary for hardware deployment.
