# -*- coding: utf_8 -*-

import bson
import math
import time


class SimpleTimer(object):
    def __init__(self, sec=0, hz=1):
        self.set_time(sec)
        self.period = 1.0 / hz

    def set_time(self, sec):
        self.tm = sec

    def elapsed(self, sec):
        return sec - self.tm

    def check_elapsed(self, sec, set_if_elapsed=True):
        elapsed = self.elapsed(sec)
        if elapsed > self.period:
            if set_if_elapsed:
                self.set_time(sec)
            return (True, elapsed)
        return (False, elapsed)


def build_ros_header(time, frame_id, seq):
    s2, s = math.modf(time)
    return {
        'stamp': {'secs': int(s), 'nsecs': int(s2 * 1000000000)},
        'frame_id': frame_id,
        'seq': seq
    }


def build_sigverse_message(topic_name, topic_type, sensor_message, op='publish'):
    if topic_name[0] != '/':
        topic_name = "/" + topic_name
    return {
        'op': op,
        'topic': topic_name,
        'type': topic_type,
        'msg': sensor_message
    }


def bson_serialize(message):
    return bson.BSON.encode(message)


def build_ros_array_msg(data):
    return {'data': data,
            'layout': {
                'dim': [{'stride': 0, 'size': 0, 'label': ''}],
                'data_offset': 0}
            }


def build_ros_odometry_2d(time, frame_id, child_frame_id, seq, x, y, yaw, vel_x, vel_y, vel_yaw):
    q = euler_to_quaternion(0, 0, yaw)
    header = build_ros_header(time, frame_id, seq)
    pose = {
        'pose': {
            'position': {'x': x, 'y': y, 'z': 0.0},
            'orientation': {'x': q[0], 'y': q[1], 'z': q[2], 'w': q[3]}
        },
        'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
    twist = {
        'twist': {
            'linear': {'x': vel_x, 'y': vel_y, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': vel_yaw}
        },
        'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
    return {
        'header': header,
        'child_frame_id': child_frame_id,
        'pose': pose,
        'twist': twist
    }


def euler_to_quaternion(ai, aj, ak):
    # https://github.com/davheld/tf/blob/master/src/tf/transformations.py
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q0 = cj * sc - sj * cs
    q1 = cj * ss + sj * cc
    q2 = cj * cs - sj * sc
    q3 = cj * cc + sj * ss
    return (q0, q1, q2, q3)


def normalize_angle(angle):
    result = math.fmod(angle + math.pi, 2.0 * math.pi)
    if result <= 0.0:
        return result + math.pi
    return result - math.pi
