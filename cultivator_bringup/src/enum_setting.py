#!/usr/bin/env python
# -*- coding: utf8 -*-
from enum import IntEnum

class CommandType(IntEnum):
    emergency_stop = 0
    no_command = 1
    moveable = 2
    mapping = 3
    cultivate = 4
    digging = 5
    auto = 6

class RobotState(IntEnum):
    emergency = 0
    standby = 1
    other = 2
    mapping = 3
    cultivate = 4

class MotorState(IntEnum):
    exception = 0
    stop = 1
    straight = 2
    curve = 3

class CultivateCmd(IntEnum):
    work_no = 0
    work_begin = 1
    work_end = 2

class CultivateFb(IntEnum):
    exception = 0
    standby = 1
    dropping = 2
    working = 3
    raising = 4
