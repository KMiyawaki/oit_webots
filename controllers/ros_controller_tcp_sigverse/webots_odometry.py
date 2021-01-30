# -*- coding: utf_8 -*-
from math import sin, cos
from utils import build_ros_odometry_2d, normalize_angle


class WebotsOdometry(object):  # for ROS coordinate system
    def __init__(self, tread, wheel_radius, pos_left_motor=0, pos_right_motor=0, frame_id='odom', child_frame_id='base_link'):
        self.tread = tread
        self.wheel_radius = wheel_radius
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.reset(pos_left_motor, pos_right_motor)

    def update_wheel(self, pos_new, pos_cur):
        d = (pos_new - pos_cur) * self.wheel_radius
        return (d, pos_new)

    def update_and_get_ros_odom(self, webots_time, pos_left_motor, pos_right_motor, delta):
        result = self.update(pos_left_motor, pos_right_motor, delta)
        ros_odom = build_ros_odometry_2d(webots_time, self.frame_id, self.child_frame_id, self.seq,
                                         result[0], result[1], result[2],
                                         result[3], 0, result[4])
        self.seq = self.seq + 1
        return ros_odom

    def update(self, pos_left_motor, pos_right_motor, delta):
        (d_left, self.pos_left_motor) = self.update_wheel(
            pos_left_motor, self.pos_left_motor)
        (d_right, self.pos_right_motor) = self.update_wheel(
            pos_right_motor, self.pos_right_motor)
        d_total = (d_left + d_right) / 2.0
        if d_right != d_left:
            d_yaw = (d_right - d_left) / self.tread
            new_yaw = self.yaw + d_yaw
            radius = d_total / d_yaw
            self.x += radius * (sin(new_yaw) - sin(self.yaw))
            self.y -= radius * (cos(new_yaw) - cos(self.yaw))
            self.yaw = normalize_angle(new_yaw)
        else:
            d_yaw = 0
            self.x += d_total * cos(self.yaw)
            self.y += d_total * sin(self.yaw)

        linear_x = d_total / delta
        angular_z = d_yaw / delta
        return (self.x, self.y, self.yaw, linear_x, angular_z)

    def reset(self, pos_left_motor, pos_right_motor):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.pos_left_motor = pos_left_motor
        self.pos_right_motor = pos_right_motor
        self.seq = 0
