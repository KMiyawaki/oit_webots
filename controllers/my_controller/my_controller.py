"""my_controller controller."""

import copy
import math
from math import sin, cos, fmod
from threading import Lock
from controller import Robot
from roslibpy import Message, Ros, Topic

"""
Download Twisted‑20.3.0‑cp39‑cp39‑win_amd64.whl from https://www.lfd.uci.edu/~gohlke/pythonlibs/#twisted
pip install Twisted‑20.3.0‑cp39‑cp39‑win_amd64.whl
pip install roslibpy
"""


def build_ros_header(webots_time, frame_id, seq):
    s2, s = math.modf(webots_time)
    return {
        'stamp': {'secs': int(s), 'nsecs': int(s2 * 1000000000)},
        'frame_id': frame_id,
        'seq': seq
    }


def build_ros_odometry_2d(webots_time, frame_id, child_frame_id, seq, x, y, yaw, vel_x, vel_y, vel_yaw):
    q = euler_to_quaternion(0, 0, yaw)
    header = build_ros_header(webots_time, frame_id, seq)
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
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
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
    result = fmod(angle + math.pi, 2.0 * math.pi)
    if result <= 0.0:
        return result + math.pi
    return result - math.pi


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


class MyController(Robot):
    # https://www.cyberbotics.com/doc/guide/pioneer-3dx?version=master
    def __init__(self, tread=0.381, wheel_radius=0.195/2):
        super(MyController, self).__init__()
        self.timestep = int(self.getBasicTimeStep())

        self.odometry = WebotsOdometry(tread, wheel_radius)

        hz = 50
        period = int(1000 / hz)
        self.left_wheel = self.getDevice("left wheel")
        self.left_wheel.setPosition(float('inf'))
        self.left_wheel_sensor = self.getDevice("left wheel sensor")
        self.left_wheel_sensor.enable(period)
        self.right_wheel = self.getDevice("right wheel")
        self.right_wheel.setPosition(float('inf'))
        self.right_wheel_sensor = self.getDevice("right wheel sensor")
        self.right_wheel_sensor.enable(period)
        # self.enum_devices()

        self.kinect_color = self.getDevice("kinect color")
        self.kinect_color.enable(self.timestep)
        self.kinect_range = self.getDevice("kinect range")
        self.kinect_range.enable(self.timestep)
        self.lrf = self.getDevice("Hokuyo UTM-30LX")
        self.lrf.enable(self.timestep)
        # const int kinect_width = wb_range_finder_get_width(kinectRange);
        # const int kinect_height = wb_range_finder_get_height(kinectRange);

        self.ros_client = None
        self.connect_ros()
        self.pub_odom = Topic(self.ros_client, '/odom', 'nav_msgs/Odometry')
        self.sub_cmd_vel = Topic(
            self.ros_client, '/cmd_vel', 'geometry_msgs/Twist')
        self.sub_cmd_vel.subscribe(self.twist_callback)
        self.tead_2 = tread / 2
        self.wheel_radius = wheel_radius
        self.twist = None
        self.twist_lock = Lock()

    def close_ros(self):
        try:
            if self.ros_client is not None:
                self.ros_client.terminate()
                self.ros_client = None
        except Exception as e:
            print(str(e))

    def connect_ros(self, ip='127.0.0.1', port=9090):
        self.close_ros()
        try:
            self.ros_client = Ros(ip, port)
            self.ros_client.run()
        except Exception as e:
            print(str(e))

    def twist_callback(self, message):
        self.twist_lock.acquire()
        self.twist = copy.deepcopy(message)
        self.twist_lock.release()

    def twist_to_vel_wheels(self):
        vel_wheels = [0, 0]  # left, right
        self.twist_lock.acquire()

        if self.twist is not None:
            lin = float(self.twist["linear"]["x"])
            ang = float(self.twist["angular"]["z"])
            # http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
            vel_wheels[0] = (lin - ang * self.tead_2) / \
                self.wheel_radius  # rad/s
            vel_wheels[1] = (lin + ang * self.tead_2) / \
                self.wheel_radius  # rad/s

        self.twist_lock.release()
        return vel_wheels

    def enum_devices(self):
        device_num = self.getNumberOfDevices()
        for i in range(0, device_num):
            dev = self.getDeviceByIndex(i)
            print(dev.getName())

    def run(self):
        # main control loop: perform simulation steps of 32 milliseconds
        # and leave the loop when the simulation is over
        while self.step(self.timestep) != -1:
            pl = self.left_wheel_sensor.getValue()
            pr = self.right_wheel_sensor.getValue()
            odom = self.odometry.update_and_get_ros_odom(
                self.getTime(), pl, pr, self.timestep / 1000.0)

            if self.ros_client is not None and self.ros_client.is_connected:
                self.pub_odom.publish(Message(odom))
                vel_wheels = self.twist_to_vel_wheels()
                self.left_wheel.setVelocity(vel_wheels[0])
                self.right_wheel.setVelocity(vel_wheels[1])
            else:
                self.connect_ros()


# main Python program
controller = MyController()
controller.run()
