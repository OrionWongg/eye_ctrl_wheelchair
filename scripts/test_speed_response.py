"""CPU-only response-analysis tests; never import ROS or publish messages."""
import unittest
from speed_response import evaluate, first_hold, segments, timing


def command(time, action, direction=None):
    return dict(time_ns=int(time*1e9), action=action, direction=direction)


def rows(end=8., start=2., release=6., vx=.4, vy=.0, w=.0):
    result = []
    for index in range(int(end*50)+1):
        stamp = index/50
        active = start+.2 <= stamp < release+.4
        result.append((stamp, 0., 0., 0., vx if active else 0., vy if active else 0., w if active else 0., .01))
    return result


class ResponseTests(unittest.TestCase):
    def test_stationary_start_plateau_and_complete_stop(self):
        trial = segments([command(2, 'keyboard_drag_begin'), command(2, 'app_target_move', 'up'), command(6, 'keyboard_drag_end')])[0]
        result = evaluate(trial, rows())
        self.assertTrue(result['start_stopped'])
        self.assertTrue(result['plateau_candidate'])
        self.assertAlmostEqual(result['steady_vx_mps'], .4)
        self.assertAlmostEqual(result['onset_delay_s'], .2)
        self.assertAlmostEqual(result['release_to_stop_s'], .4)

    def test_short_accelerating_trial_has_no_platform(self):
        trial = segments([command(2, 'keyboard_drag_begin'), command(2, 'app_target_move', 'up'), command(2.3, 'keyboard_drag_end')])[0]
        result = evaluate(trial, rows(release=2.3))
        self.assertFalse(result['plateau_candidate'])
        self.assertIsNone(result['steady_vx_mps'])

    def test_direction_change_is_not_a_release(self):
        trials = segments([command(2, 'keyboard_drag_begin'), command(2, 'app_target_move', 'up'), command(3, 'app_target_move', 'left'), command(4, 'keyboard_drag_end')])
        self.assertEqual(len(trials), 2)
        self.assertIsNone(trials[0]['release'])
        self.assertEqual(trials[0]['end_reason'], 'direction_changed_without_release')
        self.assertEqual(trials[1]['release'], 4)

    def test_truncated_hold_and_wrong_sign_do_not_qualify(self):
        trial = segments([command(2, 'keyboard_drag_begin'), command(2, 'app_target_move', 'right')])[0]
        result = evaluate(trial, rows(vx=0., w=.8))
        self.assertFalse(result['released'])
        self.assertIsNone(result['onset_delay_s'])
        self.assertFalse(result['plateau_candidate'])

    def test_gap_cannot_establish_continuous_stop(self):
        sparse = [(t, 0, 0, 0, 0, 0, 0, .01) for t in (0, .02, .04, 1.)]
        self.assertIsNone(first_hold(sparse, lambda row: True, .3))
        self.assertEqual(timing(sparse)['max_gap_s'], .96)

    def test_moving_baseline_not_clean(self):
        trial = segments([command(2, 'keyboard_drag_begin'), command(2, 'app_target_move', 'up'), command(6, 'keyboard_drag_end')])[0]
        self.assertFalse(evaluate(trial, rows(start=0.))['start_stopped'])

    def test_backward_clock_is_reported(self):
        data = [(t, 0, 0, 0, 0, 0, 0, .01) for t in (0, .02, .02, .01, .04)]
        self.assertEqual(timing(data)['duplicate_or_backward'], 2)


if __name__ == '__main__':
    unittest.main()
