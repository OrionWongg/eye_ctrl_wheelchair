import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from blink_events import BlinkEvents


class BlinkEventTest(unittest.TestCase):
    def test_individual_durations_accept_short_overlap(self):
        b=BlinkEvents(2);event=None
        for t,s in [(0.,['open','open']),(.04,['closed','open']),(.12,['closed','closed']),
                    (.17,['open','closed']),(.24,['open','open'])]:
            event,_=b.update(s,t,True)
        self.assertIsNotNone(event)
        self.assertAlmostEqual(event['durations'][0],.13)
        self.assertAlmostEqual(event['durations'][1],.12)
        self.assertLess(.17-.12,.06)  # Old simultaneous-closure timing rejected it.

    def test_unknown_without_observed_closure_never_counts(self):
        b=BlinkEvents(2)
        for t,s in [(0.,['open']*2),(.04,['unknown']*2),(.12,['open']*2)]:
            self.assertIsNone(b.update(s,t,True)[0])

    def test_short_unknown_transition_preserves_confirmed_closure(self):
        b=BlinkEvents(2)
        b.update(['open']*2,0.,True)
        b.update(['closed','open'],.04,True)
        b.update(['closed','unknown'],.08,True)
        b.update(['closed','closed'],.12,True)
        b.update(['open','closed'],.20,True)
        self.assertIsNotNone(b.update(['open']*2,.28,True)[0])

    def test_sustained_unknown_or_stale_cancels_evidence(self):
        for stale in (False,True):
            b=BlinkEvents(2);b.update(['open']*2,0.,True);b.update(['closed']*2,.04,True)
            b.update(['unknown']*2,.08,not stale)
            b.update(['unknown']*2,.24,True)
            self.assertIsNone(b.update(['open']*2,.28,True)[0])

    def test_two_separate_winks_do_not_match(self):
        b=BlinkEvents(2)
        for t,s in [(0.,['open']*2),(.04,['closed','open']),(.16,['open']*2),
                    (.20,['open','closed']),(.32,['open']*2)]:
            self.assertIsNone(b.update(s,t,True)[0])

    def test_confirmed_closure_includes_short_unknown_onset(self):
        # 68 ms samples: the left eye is ambiguous before confirmed closure.
        # Timing only from confirmation leaves 50 ms and loses the whole pair.
        b=BlinkEvents(2)
        for t,states in [(0.,['open']*2),(.068,['unknown','closed']),
                         (.136,['closed','closed'])]:
            self.assertIsNone(b.update(states,t,True)[0])
        event,_=b.update(['open']*2,.186,True)
        self.assertIsNotNone(event)
        self.assertAlmostEqual(event['durations'][0],.118)

    def test_expired_unknown_onset_cannot_be_backdated(self):
        b=BlinkEvents(2)
        for t,states in [(0.,['open']*2),(.04,['unknown']*2),
                         (.20,['closed']*2),(.30,['open']*2)]:
            self.assertIsNone(b.update(states,t,True)[0])

    def test_source_gap_reports_discontinuity_even_on_open_frame(self):
        b=BlinkEvents(2);b.update(['open']*2,0.,True)
        self.assertIsNone(b.update(['open']*2,.3,True)[0])
        self.assertFalse(b.continuity)
        b.update(['open']*2,.34,True)
        self.assertTrue(b.continuity)

    def test_long_close_is_one_stop_not_a_short_blink(self):
        b=BlinkEvents(2);b.update(['open']*2,0.,True);stops=0
        for i in range(1,25):
            event,long=b.update(['closed']*2,i*.04,True)
            self.assertIsNone(event);stops+=int(long)
        self.assertEqual(stops,1)
        self.assertIsNone(b.update(['open']*2,1.,True)[0])

if __name__=='__main__':unittest.main()
