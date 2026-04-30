from enum import IntEnum


class RobotStatus(IntEnum):
    IDLE    = 0
    MOVING  = 1
    STOPPED = 2
    REACHED = 3
    ERROR   = 4
