# Jetson Orin Nano humanoid runtime

The runtime currently tests standard forward walking with the current
`Humanoid_Robot_RSL_RL` walking/stepping ONNX interface. Vision is disconnected.

- 49 observations, including step distance and crossing command.
- Constant forward velocity: **0.4 m/s** by default; lateral velocity and yaw rate: **0**.
- Constant default step distance: **0.08 m**; crossing command: **0**.
- 50 Hz inference, 12 joint-position actions, action scale 0.25.
- No automatic stop, turn, bar-crossing command, or UDP vision listener.
- Existing motor enable, state watchdog, joint limits, and emergency shutdown remain active.

These values match training commit `4eb3d5b4d72a792c610ad46f0a8c65b931ed3b22`.
The joint order, default pose, and IMU acceleration scaling are unchanged.

## Run

Export the current walking/stepping checkpoint to ONNX and copy it to
`humanoid_jetson_deploy/models/current_walking.onnx`. Include its trained
observation normalizer in the export if normalization was enabled.
Existing model binaries are not updated in this change. Legacy 47/48-input
walking-test models are rejected; a 49-input model must also have the documented
observation semantics.

```bash
cd humanoid_jetson_deploy
python main.py --model models/current_walking.onnx --port /dev/ttyACM0
```

Motor output remains disabled unless `--enable-motors` is passed.
Follow the calibration and motor-test procedure in
[the deployment guide](humanoid_jetson_deploy/README.md).
Add `--no-plot` for headless operation; CSV position and IMU logging stays enabled.
`--max-seconds` ends a timed run and disables motors as before.

`--vx` selects a constant positive speed (at most 1 m/s), defaulting to 0.4.
`--command-source` accepts only `fixed`, and `--wz` accepts only 0.
The step distance remains the configured default for the entire run.

## Vision code

`vision/`, `connector.py`, and the reusable UDP command source remain in the
repository, but `main.py` does not instantiate a UDP receiver or consume their
output. Starting a camera/connector process cannot change this walking command.
No camera process is needed to run the policy.

## Tests

```bash
python -m unittest discover -s tests -v
cd humanoid_jetson_deploy
python -m unittest discover -s tests -v
```
