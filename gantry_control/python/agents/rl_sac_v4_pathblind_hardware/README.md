# RL SAC V4 Pathblind Hardware

This package trains and evaluates the path-blind SAC policy directly on hardware.
It is intentionally hardware-only.
There is no simulator backend in this folder.

The code is designed for our current setup:
- one physical tank
- one episode at a time
- CPU-only execution on a local machine
- policy fixed during an episode
- learning updates run between episodes
- restartable training through checkpoints and replay snapshots

## 1. What This Package Is

This folder contains the hardware deployment and hardware training version of the path-blind SAC agent.
Its job is to:
- load target paths
- command hardware velocities `vx` and `vy`
- collect whisker sensor frames and state estimates
- turn hardware data into SAC observations, rewards, and termination flags
- store valid transitions in replay
- update the SAC policy between episodes
- save checkpoints, plots, and logs so training can be resumed after interruptions

This package does not contain any simulation environment logic.
If something here still looks simulation-like, it is only because SAC still needs the same RL pieces: observations, actions, rewards, replay, checkpoints, and evaluation.

## 2. Recommended Reading Order

- `README.md`: high-level workflow and assumptions
- `config.py`: important defaults and safety parameters
- `hardware_runtime_stub.py`: how to implement the real hardware runtime
- `hardware_adapter.py`: how runtime outputs become rewards, terminations, and observations
- `train.py`: full training loop, checkpointing, replay writes, and between-episode updates
- `evaluate.py`: deterministic rollout for testing a checkpoint

## 3. Main Files

- `config.py`: central configuration and default values
- `hardware_adapter.py`: hardware environment wrapper used by the trainer
- `hardware_runtime_stub.py`: template showing how a real hardware runtime should look
- `train.py`: training entry point
- `evaluate.py`: evaluation entry point
- `models.py`: SAC actor and critics
- `replay_buffer.py`: replay storage and sampling
- `plot_utils.py`: per-episode plots
- `analyze_run.py`: post-run analysis of CSV logs

## 4. Core Design Choices

The most important design decisions are:
- single tank only, no parallel envs
- one path is run per episode
- the policy does not change in the middle of an episode
- replay is filled during the episode
- SAC updates happen only after the episode ends
- hardware or I/O failures are logged but skipped from replay
- training is restartable from checkpoints and saved replay

This means the training loop is intentionally different from the fast simulation setup.
On hardware, stability and recoverability matter more than squeezing out updates during live execution.

## 5. Important Defaults

Current defaults in `config.py` include:
- `path_subs`: `path1` to `path16`
- `eval_path_subs`: `path1`, `path5`, `path9`, `path13`
- `fixed_x_speed_mm_per_ms`: `0.2`
- `object_tangential_speed_mm_per_ms`: `0.2`
- `episode_time_ms`: `20000`
- `finish_line_mm`: `3400`
- `initial_object_gap_mm`: `200`
- `min_object_x_gap_terminate_mm`: `25`
- `terminate_corridor_half_width_mm`: `300`
- `target_update_to_data_ratio`: `0.5`
- `replay_size`: `50000`
- `batch_size`: `128`
- `total_env_steps`: `10000000`
- `device`: `cpu`

These are defaults only.
Training and evaluation commands can override them through CLI flags.

## 6. Units and Conventions

The runtime and the trainer use the following units:
- position: millimeters
- velocity: millimeters per millisecond
- time: milliseconds

Important examples:
- `0.2 mm/ms = 0.2 m/s`
- `episode_time_ms = 20000` means a 20 second episode

The trainer assumes:
- `time_ms` starts near 0 at the beginning of each episode
- `x_mm`, `y_mm`, `vx_mm_per_ms`, `vy_mm_per_ms` are all in the same coordinate frame
- target paths are stored as `N x 2` arrays of `[x_mm, y_mm]`

## 7. Observation and Action Meaning

### Action

The SAC policy outputs one scalar action.
The adapter converts it into commanded hardware motion:
- `vx` is fixed by `fixed_x_speed_mm_per_ms`
- the policy controls lateral motion through `vy`

So the runtime receives:
- `cmd_vx_mm_per_ms`
- `cmd_vy_mm_per_ms`
- `hold_frames`

### Observation

The policy observes two pieces:
- whisker sensor history
- kinematic features

Sensor history shape:
- `(history_steps, 3, 3, num_signal_channels)`

Kinematic feature vector contents:
- `x_mm`
- `y_mm`
- `vx_mm_per_ms`
- `vy_mm_per_ms`
- previous commanded `vy`
- normalized episode time

## 8. Runtime Interface You Must Implement

The trainer does not talk directly to lab-specific drivers.
Instead, it loads a runtime factory from:
- `cfg.hardware_runtime_factory = "module:function"`

That factory is called as:

```python
runtime = function(cfg)
```

The returned runtime object must implement:
- `start_episode(spec) -> HardwareResetResult`
- `step(cmd_vx_mm_per_ms, cmd_vy_mm_per_ms, hold_frames) -> HardwareStepResult`
- `close() -> None`

The best reference for this contract is:
- `hardware_adapter.py`
- `hardware_runtime_stub.py`

## 9. What `start_episode(spec)` Must Do

`start_episode(spec)` is called once at the beginning of each episode.
Its job is to:
- prepare the hardware for a fresh rollout
- move or confirm the start state
- begin sensor collection
- return the initial sensor frames and pose

`spec` contains the episode setup chosen by the trainer, including at least:
- `episode_index`
- `path_sub`
- `episode_time_ms`
- `sensor_frame_period_ms`
- `rl_interval`
- `start_x_mm`
- `start_y_mm`
- `fixed_x_speed_mm_per_ms`
- `y_speed_limit_mm_per_ms`
- `vel_max_mm_per_ms`
- `object_tangential_speed_mm_per_ms`
- `initial_object_gap_mm`
- `min_object_x_gap_terminate_mm`

It must return a `HardwareResetResult` containing:
- `sensor_frames`
- `pose`
- optional `info`

Expected sensor frame shape:
- either `(3, 3, C)` for one frame
- or `(N, 3, 3, C)` for multiple frames

Expected pose contents:
- `x_mm`
- `y_mm`
- `vx_mm_per_ms`
- `vy_mm_per_ms`
- `time_ms`

If reset truly fails, raise an exception.
The trainer will retry according to `reset_retry_attempts`.

## 10. What `step(...)` Must Do

`step(cmd_vx_mm_per_ms, cmd_vy_mm_per_ms, hold_frames)` is called once per RL control interval.
Its job is to:
- apply the commanded `vx` and `vy`
- hold or stream that command for `hold_frames` sensor frames
- collect the sensor frames produced during that interval
- return the newest pose and any runtime status flags

It must return a `HardwareStepResult` containing:
- `sensor_frames`: new frames gathered during this control interval
- `pose`: latest pose after the interval
- `done`: runtime-level terminal flag if your runtime itself ended the episode
- `truncated`: runtime-level non-task stop flag
- `info`: optional extra measurements or failure flags

Typical sensor frame shape for one step in your current setup:
- `(4, 3, 3, 2)`

Why `4`?
- `rl_interval = 4`
- one RL step holds the command for 4 sensor frames
- with `sensor_frame_period_ms = 12.5`, this is one 50 ms control interval

Returning all frames from the interval is better than returning only one final frame, because the history buffer can then preserve the whisker dynamics within the interval.

## 11. What `step_out` Contains

Inside `hardware_adapter.py`, the code does:

```python
step_out = self.runtime.step(
    cmd_vx_mm_per_ms=cmd_vx,
    cmd_vy_mm_per_ms=cmd_vy,
    hold_frames=int(self.cfg.rl_interval),
)
```

`step_out` is a `HardwareStepResult` object, not a raw NumPy array.
It contains:
- `step_out.sensor_frames`
- `step_out.pose`
- `step_out.done`
- `step_out.truncated`
- `step_out.info`

So `step_out` itself has no array shape.
The array shape belongs to:
- `step_out.sensor_frames`

## 12. `done` vs `truncated`

Both end the episode, but they mean different things.

- `done`: the episode ended because of a terminal condition
- `truncated`: the episode was cut off for an external or non-task reason

In the current hardware adapter:
- `done` includes `too_far`, `too_close`, or `runtime_done`
- `truncated` includes time limit or `runtime_truncated`

In the current trainer, both are treated the same for replay bootstrapping because they are merged into one final `done` flag before being stored.
The distinction is still useful for logs, debugging, and future refinement.

## 13. Safety and Termination Logic

The hardware adapter computes task-level reward and termination.
Your runtime does not need to reproduce all of this logic.

Important task-level checks include:
- `too_far`: lateral error exceeded the allowed corridor
- `too_close`: object x-gap dropped below `min_object_x_gap_terminate_mm`
- `time_limit_reached`: episode time exceeded `episode_time_ms`
- `finish_line_reached`: current `x_mm` passed `finish_line_mm`

The adapter also accepts runtime-side failure flags from `info`, such as:
- `command_failed`
- `sensor_timeout`
- `state_timeout`
- `hardware_error`
- `infrastructure_failure`

If your lab system has its own emergency stop, that is still good and should remain independent of the RL code.

## 14. Why `valid_replay_mask` Exists

Not every hardware step is suitable for learning.
If a transition came from a hardware or I/O failure, it should usually not go into replay.

The training loop marks a step invalid for replay when there is an infrastructure failure such as:
- command failure
- sensor timeout
- state timeout
- hardware error

This is done through `valid_replay_mask`.
Conceptually it means:
- `True`: store this transition in replay
- `False`: skip this transition

This protects SAC from learning from corrupted or non-physical transitions.

## 15. Training Flow

A normal episode looks like this:
1. select one training path
2. call `runtime.start_episode(spec)`
3. build the initial observation history
4. repeatedly choose an action and call `runtime.step(...)`
5. store valid transitions in replay
6. end the episode when `done` or `truncated` becomes true
7. run SAC updates between episodes
8. save logs, plots, checkpoints
9. optionally wait for hardware settling time
10. start the next episode

This is the most important behavior to understand:
- policy weights stay fixed during the episode
- policy updates happen only after the episode ends

## 16. Checkpointing and Restart

The training code is designed to survive interruptions.
A checkpoint includes:
- actor weights
- critic weights
- target networks
- optimizer state
- entropy temperature state
- replay buffer path
- RNG state
- counters such as `total_env_steps` and `episodes_completed`

Saved files include:
- `checkpoint_*.pt`
- `replay_*.npz`
- `latest_checkpoint.pt`
- `latest_replay.npz`
- `final_model.pt`
- `final_replay.npz`

If hardware fails or the process stops, training can be resumed.

## 17. Training

Training is meant to run locally, not on Expanse.
Run commands from the parent directory of `rl_sac_v4_pathblind_hardware`, not from inside the package folder.

Minimal training run using config defaults:

```bash
python -u -m rl_sac_v4_pathblind_hardware.train
```

Training run with the most common overrides:

```bash
python -u -m rl_sac_v4_pathblind_hardware.train \
  --runtime-factory my_hardware.runtime:create_runtime \
  --target-paths-root /absolute/path/to/hardware_target_paths \
  --path-subs path1,path2,path3,path4,path5,path6,path7,path8,path9,path10,path11,path12,path13,path14,path15,path16 \
  --eval-path-subs path1,path5,path9,path13 \
  --output-dir rl_sac_v4_pathblind_hardware_runs/run_hw_01
```

For one-off changes, pass the flag directly. Example:

```bash
python -u -m rl_sac_v4_pathblind_hardware.train \
  --episode-time-ms 25000 \
  --settle-time-seconds 60
```

## 18. Resume Training

Resume from the latest checkpoint in an existing run directory:

```bash
python -u -m rl_sac_v4_pathblind_hardware.train \
  --resume \
  --output-dir rl_sac_v4_pathblind_hardware_runs/run_hw_01
```

Resume from a specific checkpoint:

```bash
python -u -m rl_sac_v4_pathblind_hardware.train \
  --resume \
  --resume-path rl_sac_v4_pathblind_hardware_runs/run_hw_01/checkpoint_0001200.pt \
  --output-dir rl_sac_v4_pathblind_hardware_runs/run_hw_01
```

## 19. Evaluation

Evaluation is separate from training and is deterministic.
Use it when you want to:
- test a saved checkpoint
- evaluate on prescribed eval paths
- test on unseen paths later

Minimal evaluation command:

```bash
python -u -m rl_sac_v4_pathblind_hardware.evaluate \
  rl_sac_v4_pathblind_hardware_runs/run_hw_01/final_model.pt
```

Evaluation with the most common overrides:

```bash
python -u -m rl_sac_v4_pathblind_hardware.evaluate \
  rl_sac_v4_pathblind_hardware_runs/run_hw_01/final_model.pt \
  --runtime-factory my_hardware.runtime:create_runtime \
  --target-paths-root /absolute/path/to/hardware_target_paths \
  --path-sub path1 \
  --output-dir rl_sac_v4_pathblind_hardware_runs/run_hw_01/eval_final_model
```

## 20. Output Files

A run directory typically contains:
- `config.json`: snapshot of run settings
- `episode_log.csv`: one row per episode
- `train_log.csv`: one row per episode with training metrics
- `checkpoint_*.pt`: intermediate checkpoints
- `replay_*.npz`: replay snapshots matching checkpoints
- `latest_checkpoint.pt`
- `latest_replay.npz`
- `final_model.pt`
- `final_replay.npz`
- `episodes/episode_xxxxxx/rollout.csv`: step-by-step rollout data
- `episodes/episode_xxxxxx/trajectory.png`: target path vs actual trajectory
- `episodes/episode_xxxxxx/metrics.png`: rollout metrics plot

## 21. Common Questions

### Why are updates not happening yet?

Because updates do not start until replay is large enough.
The trainer waits until replay size reaches at least:
- `max(update_after, batch_size)`

### Why are some steps missing from replay?

Because steps with infrastructure failures are intentionally excluded from replay.
That prevents learning from corrupted transitions.

### Why is the policy not changing during the episode?

This is deliberate.
For hardware, it is cleaner and safer to keep the policy fixed during a rollout and update only between episodes.

### Why is there still a replay buffer if this is hardware-only?

Because SAC is still an off-policy algorithm.
Hardware-only does not remove the need for replay, critic targets, checkpoints, or batch updates.
It only removes the simulation backend.

## 22. Common Problems

- Target path file not found:
  check `TARGET_PATHS_ROOT`, `PATH_SUBS`, and `PATH_FILE_TEMPLATE`.
- Runtime import fails:
  check `RUNTIME_FACTORY` format `module:function` and `PYTHONPATH`.
- Replay updates never start:
  replay may still be below the warm-up threshold.
- Many replay entries are skipped:
  the runtime is reporting too many infrastructure failures.
- Sensor frame shape error:
  your runtime should return `(3,3,C)` or `(N,3,3,C)`.
- Wrong motion scale:
  check that all positions are in mm and velocities are in mm/ms.

## 23. What a New Developer Usually Needs to Change

A new developer usually only needs to change these things first:
- implement a real runtime module with `create_runtime(cfg)`
- point `hardware_runtime_factory` to that module
- set `TARGET_PATHS_ROOT` to the correct path library
- choose training paths in `PATH_SUBS`
- choose evaluation paths in `eval_path_subs` or by passing `--path-sub` to evaluation
- set settling time and any runtime-specific timeout values

The SAC model, replay buffer, and training loop should usually not need structural changes for initial bring-up.

## 24. Final Notes

This folder is meant to be a clean hardware training package.
It should remain free of simulator-specific code.
When in doubt, keep lab-specific device logic inside the runtime implementation and keep RL logic in this package.
