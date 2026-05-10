import os, sys, threading, time, types, unittest
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)

from pathfinder.ros.pose_tracker import PoseTracker


class TestPoseTracker(unittest.TestCase):
    """Unit tests for PoseTracker."""

    def _make_pose(self, x=1.0, y=2.0, theta=0.0):
        return types.SimpleNamespace(x=x, y=y, theta=theta)

    def test_update_then_wait_for_returns_stored_pose(self):
        tracker = PoseTracker(timeout_sec=1.0)
        pose = self._make_pose()
        tracker.update('tb3_0', pose)
        result = tracker.wait_for('tb3_0')
        self.assertIs(result, pose)

    def test_wait_for_blocks_until_update_from_background_thread(self):
        tracker = PoseTracker(timeout_sec=2.0)
        pose = self._make_pose(x=3.0)

        def delayed_update():
            time.sleep(0.05)
            tracker.update('tb3_1', pose)

        thread = threading.Thread(target=delayed_update, daemon=True)
        thread.start()
        result = tracker.wait_for('tb3_1')
        thread.join()
        self.assertIs(result, pose)

    def test_wait_for_returns_none_on_timeout(self):
        tracker = PoseTracker(timeout_sec=0.05)
        result = tracker.wait_for('tb3_2')
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
