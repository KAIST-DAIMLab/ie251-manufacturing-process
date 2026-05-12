from __future__ import annotations
import os
import sys
import types
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, ROOT)


from pathfinder.safety.forward_gate import ForwardGate


def _twist(linear_x=0.0, angular_z=0.0):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=linear_x),
        angular=types.SimpleNamespace(z=angular_z),
    )


class FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class ForwardGateTest(unittest.TestCase):

    def test_unblocked_passes_forward_velocity_unchanged(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)

        gate.publish(_twist(linear_x=0.22))

        self.assertEqual(pub.published[-1].linear.x, 0.22)

    def test_blocked_zeros_positive_linear_x(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)
        gate.set_blocked(True)

        gate.publish(_twist(linear_x=0.22))

        self.assertEqual(pub.published[-1].linear.x, 0.0)

    def test_blocked_passes_angular_z_unchanged(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)
        gate.set_blocked(True)

        gate.publish(_twist(linear_x=0.22, angular_z=1.5))

        self.assertEqual(pub.published[-1].angular.z, 1.5)

    def test_blocked_does_not_clamp_zero_linear_x(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)
        gate.set_blocked(True)

        gate.publish(_twist(linear_x=0.0, angular_z=1.5))

        self.assertEqual(pub.published[-1].linear.x, 0.0)
        self.assertEqual(pub.published[-1].angular.z, 1.5)

    def test_blocked_does_not_clamp_negative_linear_x(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)
        gate.set_blocked(True)

        gate.publish(_twist(linear_x=-0.10))

        self.assertEqual(pub.published[-1].linear.x, -0.10)

    def test_unblocking_restores_forward_velocity(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)
        gate.set_blocked(True)
        gate.set_blocked(False)

        gate.publish(_twist(linear_x=0.22))

        self.assertEqual(pub.published[-1].linear.x, 0.22)

    def test_publish_delegates_to_wrapped_publisher(self):
        pub = FakePublisher()
        gate = ForwardGate(pub)

        gate.publish(_twist())
        gate.publish(_twist())

        self.assertEqual(len(pub.published), 2)


if __name__ == '__main__':
    unittest.main()
