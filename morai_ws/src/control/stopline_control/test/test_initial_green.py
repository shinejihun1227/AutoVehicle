"""An acquired stop line always requires confirmed permission, including startup."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from stopline_control.core import StopLineControllerCore


class InitialGreenTest(unittest.TestCase):
    def test_one_green_cannot_bypass_a_new_stopline(self):
        core = StopLineControllerCore()
        core.observe_signal("GREEN", .9, True, 10., 10., 10.)
        core.observe_line(.5, .9, True, 10., 10., 10.)
        self.assertEqual(core.update(10., 10., 0.).mode, "HOLD")
        core.observe_signal("GREEN", .9, True, 10.1, 10.1, 10.1)
        self.assertEqual(core.update(10.1, 10.1, 0.).brake, 1.)
        core.observe_signal("GREEN", .9, True, 10.3, 10.3, 10.3)
        self.assertEqual(core.update(10.3, 10.3, 0.).mode, "NOMINAL")

    def test_preconfirmed_green_allows_approach_without_unnecessary_stop(self):
        core = StopLineControllerCore()
        core.observe_signal("GREEN", .9, True, 10., 10., 10.)
        core.update(10., 10., 1.)
        core.observe_signal("GREEN", .9, True, 10.3, 10.3, 10.3)
        core.observe_line(4., .9, True, 10.3, 10.3, 10.3)
        self.assertEqual(core.update(10.3, 10.3, 1.).mode, "NOMINAL")


if __name__ == "__main__":
    unittest.main()
