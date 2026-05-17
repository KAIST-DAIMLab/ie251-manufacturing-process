from enum import IntEnum


class RobotMode(IntEnum):
    """Wire-stable robot status values.

    OFFLINE is assigned by the executor when /odom goes stale (robot unreachable).
    The Web UI also synthesises it locally when the /<ns>/state heartbeat goes stale
    (executor itself died).
    """

    OFFLINE  = 0
    IDLE     = 1
    MOVING   = 2
    OBSTACLE = 3
