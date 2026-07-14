import unittest

from xbox_drive_core import joystick_to_drive_targets


class JoystickDriveTargetsTest(unittest.TestCase):
    def test_forward_max_steer_keeps_inside_track_moving_forward(self) -> None:
        _v, _w, left_mps, right_mps = joystick_to_drive_targets(1.0, 1.0)

        self.assertAlmostEqual(left_mps, 0.60, places=3)
        self.assertAlmostEqual(right_mps, 0.24, places=3)
        self.assertGreater(right_mps, 0.0)

    def test_reverse_max_steer_keeps_both_tracks_in_reverse(self) -> None:
        _v, _w, left_mps, right_mps = joystick_to_drive_targets(-1.0, 1.0)

        self.assertAlmostEqual(left_mps, -0.24, places=3)
        self.assertAlmostEqual(right_mps, -0.60, places=3)
        self.assertLess(left_mps, 0.0)

    def test_stopped_steering_preserves_rotate_in_place(self) -> None:
        _v, _w, left_mps, right_mps = joystick_to_drive_targets(0.0, 1.0)

        self.assertAlmostEqual(left_mps, 0.60, places=3)
        self.assertAlmostEqual(right_mps, -0.60, places=3)

    def test_inner_ratio_can_be_tuned_for_wider_turns(self) -> None:
        _v, _w, left_mps, right_mps = joystick_to_drive_targets(
            1.0,
            1.0,
            moving_inner_ratio=0.60,
        )

        self.assertAlmostEqual(left_mps, 0.60, places=3)
        self.assertAlmostEqual(right_mps, 0.36, places=3)


if __name__ == "__main__":
    unittest.main()
