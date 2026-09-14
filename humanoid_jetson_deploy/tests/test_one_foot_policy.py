from __future__ import annotations

import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import config
import main
from one_foot_policy import OneFootCommand, OneFootPolicy
from protocol import COMMAND_ENABLE, COMMAND_ESTOP, STATE_ENCODERS_VALID, STATE_IMU_VALID


class OneFootPolicyTests(unittest.TestCase):
    def make_policy(self, support="right", width=46, batch=1):
        with patch("policy_runner.ort.InferenceSession") as factory:
            session = factory.return_value
            session.get_inputs.return_value = [
                SimpleNamespace(name="obs", type="tensor(float)", shape=[batch, width])
            ]
            session.get_outputs.return_value = [SimpleNamespace(name="actions")]
            session.run.return_value = [np.arange(12, dtype=np.float32).reshape(1, 12)]
            return OneFootPolicy("test.onnx", support), session

    def values(self):
        return dict(
            accel_m_s2=np.array([1., 2., 9.81]),
            gyro_rad_s=np.array([4., 5., 6.]),
            projected_gravity=np.array([0.1, 0.2, -0.97]),
            lift_command=1.,
            joint_position_policy=config.Q_DEFAULT + np.arange(12) / 100,
            joint_velocity_policy=np.arange(12) + 20.,
        )

    def test_right_support_layout_actions_and_history(self):
        policy, session = self.make_policy()
        target, action, obs, _ = policy.step(**self.values())
        expected = np.r_[
            [0.1, 0.2, 0.981, 4, 5, 6, 0.1, 0.2, -0.97, 1],
            np.arange(12) / 100, np.arange(12) + 20, np.zeros(12),
        ]
        np.testing.assert_allclose(obs, expected, atol=1e-7)
        self.assertEqual(obs.dtype, np.float32)
        np.testing.assert_allclose(target, config.Q_DEFAULT + 0.25 * np.arange(12))
        np.testing.assert_array_equal(session.run.call_args.args[1]["obs"], obs[None])
        np.testing.assert_array_equal(policy.build_observation(**self.values())[34:], action)
        policy.reset()
        np.testing.assert_array_equal(policy.build_observation(**self.values())[34:], np.zeros(12))

    def test_left_support_mirrors_sensors_joints_and_targets_but_not_history(self):
        policy, _ = self.make_policy("left")
        values = self.values()
        originals = {k: v.copy() for k, v in values.items() if isinstance(v, np.ndarray)}
        for command in (1., 0.):
            values["lift_command"] = command
            target, action, obs, _ = policy.step(**values)
            np.testing.assert_allclose(obs[:10],
                [0.1, -0.2, 0.981, -4, 5, -6, 0.1, -0.2, -0.97, command], atol=1e-7)
            np.testing.assert_allclose(obs[10:22],
                [-.06, -.07, -.08, -.09, -.10, -.11, 0, -.01, -.02, -.03, -.04, -.05], atol=1e-7)
            np.testing.assert_allclose(obs[22:34],
                [-26, -27, -28, -29, -30, -31, -20, -21, -22, -23, -24, -25])
            np.testing.assert_allclose(target, config.Q_DEFAULT + 0.25 * np.array(
                [-6, -7, -8, -9, -10, -11, 0, -1, -2, -3, -4, -5]))
            np.testing.assert_array_equal(policy.build_observation(**values)[34:], action)
        for name, value in originals.items():
            np.testing.assert_array_equal(values[name], value)

    def test_model_width_and_invalid_values(self):
        for width in (47, 48, 49):
            with self.subTest(width=width), self.assertRaisesRegex(RuntimeError, "46"):
                self.make_policy(width=width)
        self.make_policy(batch="batch")
        policy, _ = self.make_policy()
        for command in (-1, .5, 2, float("nan")):
            with self.assertRaises(ValueError):
                policy.build_observation(**(self.values() | {"lift_command": command}))
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            policy.build_observation(**(self.values() | {"gyro_rad_s": [0, float("nan"), 0]}))
        with self.assertRaisesRegex(ValueError, "shape"):
            policy.build_observation(**(self.values() | {"joint_velocity_policy": np.zeros(11)}))

    def test_command_boundaries_and_no_automatic_repeat(self):
        schedule = OneFootCommand()
        self.assertEqual([schedule.get(t) for t in (0, .999, 1, 4.999, 5, 8, 100)],
                         [0, 0, 1, 1, 0, 0, 0])
        self.assertEqual(OneFootCommand(2, 3).get(1), 0)
        for value in (0, -1, float("nan"), float("inf")):
            for kwargs in ({"stand_seconds": value}, {"lift_seconds": value}):
                with self.assertRaises(ValueError):
                    OneFootCommand(**kwargs)

    def test_runtime_sequence_uses_no_walking_commands_and_disables_on_fault(self):
        with patch("sys.argv", ["main.py", "--policy", "one-foot", "--model", "test.onnx",
                                "--no-plot", "--enable-motors"]):
            args = main.parse_args()
        state = SimpleNamespace(
            status_flags=STATE_ENCODERS_VALID | STATE_IMU_VALID,
            accel_m_s2=np.array([0, 0, 9.81], dtype=np.float32),
            gyro_rad_s=np.zeros(3, dtype=np.float32),
            orientation_wxyz=np.array([1, 0, 0, 0], dtype=np.float32),
            joint_position=config.Q_DEFAULT.copy(), joint_velocity=np.zeros(12), sequence=1,
        )
        policy, session = self.make_policy()
        with (
            patch.object(main, "parse_args", return_value=args),
            patch.object(main.signal, "signal"),
            patch.object(main, "OneFootPolicy", return_value=policy),
            patch.object(main, "HumanoidPolicy", side_effect=AssertionError("walking policy")),
            patch.object(main, "FixedCommandSource", side_effect=AssertionError("velocity source")),
            patch("command_source.socket.socket", side_effect=AssertionError("vision socket")),
            patch.object(main, "SerialLink") as link_cls,
            patch.object(main, "PositionCsvLogger"),
            patch.object(main.time, "monotonic", side_effect=[0, 0, 0, 1, 1, 5, 5, 6]),
            patch.object(main.time, "sleep"),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            link = link_cls.return_value
            link.wait_for_state.return_value = state
            link.get_latest_state.side_effect = [state, state, state, RuntimeError("stale state")]
            self.assertEqual(main.main(), 1)
            inputs = [call.args[1]["obs"] for call in session.run.call_args_list[1:]]
            self.assertEqual([obs[0, 9] for obs in inputs], [0, 1, 0])
            self.assertTrue(all(obs.shape == (1, 46) for obs in inputs))
            flags = [call.args[-1] for call in link.send_command.call_args_list]
            self.assertEqual(flags[:3], [COMMAND_ENABLE] * 3)
            self.assertIn(COMMAND_ESTOP, flags[3:])
            self.assertEqual(flags[-1], 0)
            link.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
