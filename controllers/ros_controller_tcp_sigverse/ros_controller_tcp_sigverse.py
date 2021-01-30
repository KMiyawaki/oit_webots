# -*- coding: utf_8 -*-

import copy
import math
import socket

from controller import Robot
from threading import Lock

from rosbridge_sigverse import RosBridgeSIGVerse
from rosbridge_tcp import RosBridgeTCP
from utils import build_ros_array_msg, SimpleTimer, build_sigverse_message, bson_serialize
from lidar_ros import LidarRos
from webots_odometry import WebotsOdometry


class MyController(Robot):
    # https://www.cyberbotics.com/doc/guide/pioneer-3dx?version=master
    def __init__(self, tread=0.381, wheel_radius=0.195/2):
        super(MyController, self).__init__()
        self.timestep = int(self.getBasicTimeStep())

        wheel_hz = 40.0
        period = int(1000 / wheel_hz)
        self.left_wheel = self.getDevice("left wheel")
        self.left_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0)
        self.left_wheel_sensor = self.getDevice("left wheel sensor")
        self.left_wheel_sensor.enable(period)
        self.right_wheel = self.getDevice("right wheel")
        self.right_wheel.setPosition(float('inf'))
        self.right_wheel.setVelocity(0)
        self.right_wheel_sensor = self.getDevice("right wheel sensor")
        self.right_wheel_sensor.enable(period)

        sensor_hz = 30.0
        period = int(1000 / sensor_hz)
        self.kinect_color = self.getDevice("kinect color")
        self.kinect_color.enable(period)
        self.kinect_range = self.getDevice("kinect range")
        self.kinect_range.enable(period)
        self.lidar = self.getDevice("Hokuyo URG-04LX")
        self.lidar.enable(period)
        self.lidar_ros = LidarRos(self.lidar)

        self.odometry = WebotsOdometry(tread=0.381, wheel_radius=0.195/2)
        self.pub_wheel_pos_timer = SimpleTimer(self.getTime(), wheel_hz)

        self.wheel_vels = None
        self.wheel_vels_lock = Lock()

        self.sigverse_bridge = RosBridgeSIGVerse()
        self.pub_scan_timer = SimpleTimer(self.getTime(), sensor_hz)

        self.ros_bridge_tcp = RosBridgeTCP()
        advertise_msg = {
            "op": "advertise",
            "topic": "/webots/wheel_pos",
            "type": "std_msgs/Float32MultiArray"
        }
        self.ros_bridge_tcp.send_message(advertise_msg)
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/webots/wheel_vels",
            "type": "std_msgs/Float32MultiArray"
        }
        self.ros_bridge_tcp.send_message(subscribe_msg)

    def close_ros(self):
        try:
            if self.ros_bridge_tcp is not None:
                self.ros_bridge_tcp.terminate()
                self.ros_bridge_tcp = None
        except Exception as e:
            print(str(e))
        try:
            if self.sigverse_bridge is not None:
                self.sigverse_bridge.terminate()
                self.sigverse_bridge = None
        except Exception as e:
            print(str(e))

    def wheel_vels_callback(self, message):
        self.wheel_vels_lock.acquire()
        self.wheel_vels = copy.deepcopy(message)
        self.wheel_vels_lock.release()

    def set_wheel_vels_from_rosmsg(self):
        self.wheel_vels_lock.acquire()
        if self.wheel_vels is not None:
            self.left_wheel.setVelocity(self.wheel_vels["data"][0])
            self.right_wheel.setVelocity(self.wheel_vels["data"][1])
        self.wheel_vels_lock.release()

    def publish_wheel_positions(self, tm):
        (need_to_publish, _) = self.pub_wheel_pos_timer.check_elapsed(tm)
        if need_to_publish:
            pl = self.left_wheel_sensor.getValue()
            pr = self.right_wheel_sensor.getValue()
            pub_msg = {
                "op": "publish",
                "topic": "/webots/wheel_pos",
                "msg": build_ros_array_msg([pl, pr])
            }
            self.ros_bridge_tcp.send_message(pub_msg)

    def publish_scan(self, tm):
        # https://github.com/cyberbotics/webots/blob/114ef740e47613c8235022716d7cf0782e383f3a/projects/default/controllers/ros/RosLidar.cpp#L102
        (need_to_publish, _) = self.pub_scan_timer.check_elapsed(tm)
        if need_to_publish:
            scan = self.lidar_ros.get_ros_msg()
            message = build_sigverse_message(
                "/base_scan", 'sensor_msgs/LaserScan', scan)
            self.sigverse_bridge.send_message(message)

    def enum_devices(self):
        device_num = self.getNumberOfDevices()
        for i in range(0, device_num):
            dev = self.getDeviceByIndex(i)
            print(dev.getName())

    def run(self):
        # main control loop: perform simulation steps of 32 milliseconds
        # and leave the loop when the simulation is over
        while self.step(self.timestep) != -1:
            tm = self.getTime()
            if True:
                self.publish_wheel_positions(tm)
                self.publish_scan(tm)
                recved = self.ros_bridge_tcp.wait()
                for r in recved:
                    self.left_wheel.setVelocity(r["msg"]["data"][0])
                    self.right_wheel.setVelocity(r["msg"]["data"][1])
            else:
                self.connect_ros()


# main Python program
controller = MyController()
controller.run()
