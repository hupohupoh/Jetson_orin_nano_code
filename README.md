# Jetson Orin Nano humanoid runtime

The runtime supports standard forward walking from `Humanoid_Robot_RSL_RL`
and one-foot standing from `Humanoid_Robot_One_Foot_Standing`. Select the
interface with `--policy walking` (default) or `--policy one-foot`.
Vision is disconnected in both modes.

Walking defaults:

- 49 observations, including step distance and crossing command.
- Constant forward velocity: **0.4 m/s** by default; lateral velocity and yaw rate: **0**.
- Constant default step distance: **0.08 m**; crossing command: **0**.
- 50 Hz inference, 12 joint-position actions, action scale 0.25.
- No automatic stop, turn, bar-crossing command, or UDP vision listener.
- Existing motor enable, state watchdog, joint limits, and emergency shutdown remain active.

These values match training commit `4eb3d5b4d72a792c610ad46f0a8c65b931ed3b22`.
The joint order, default pose, and IMU acceleration scaling are unchanged.

## Run walking

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

## Run one-foot standing

Export your one-foot standing checkpoint and copy it to
`humanoid_jetson_deploy/models/one_foot.onnx`. This model is not bundled.
From the repository root:

```bash
cd humanoid_jetson_deploy
python main.py --policy one-foot --model models/one_foot.onnx --port /dev/ttyACM0 --enable-motors --max-seconds 8
```

The default sequence is command **0** for 1 second, **1** for 4 seconds,
then **0** until exit. This commands the left foot to lift with right-foot
support. Use `--stand-seconds` and `--lift-seconds` to change the durations.
There is one sequence per process run; it does not repeat or reset the physical
robot at 8 seconds. `--max-seconds 8` ends this example after the lowering phase
and disables motors. Omit it to continue the standing command until Ctrl+C.

`--support-foot left` selects left support/right lift with the training code's
observation and action mirroring. Right support is the default because current
training disables left/right switching. The side remains fixed throughout a run.
`--vx`, `--wz`, and `--command-source` do not supply observations to this mode.
Add `--no-plot` for headless operation.

The 46-input layout and side transforms match training commit
`c1b4e8c8bdedafc8c7fd4c162a7c3c9e0a28df9f`; see
[the one-foot interface](humanoid_jetson_deploy/README.md#one-foot-standing-interface)
for the exact order. The runtime rejects a walking model selected for one-foot
mode before opening the serial port.

Hardware testing is complete as confirmed by the owner:
`CALIBRATION_CONFIRMED = True` and `IMU_CALIBRATION_CONFIRMED = True` are
already set in `config.py` and remain so. `--enable-motors` enables motor
output for the requested run; omitting it keeps the existing dry-run behavior.

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

