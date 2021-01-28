"""my_controller controller."""

import copy
import math
from controller import Robot
from threading import Lock
from roslibpy import Message, Ros, Topic
from utils import build_ros_odometry_2d, build_ros_array_msg, SimpleTimer

"""
Download Twisted‑20.3.0‑cp39‑cp39‑win_amd64.whl from https://www.lfd.uci.edu/~gohlke/pythonlibs/#twisted
pip install Twisted‑20.3.0‑cp39‑cp39‑win_amd64.whl
pip install roslibpy
"""


class MyController(Robot):
    # https://www.cyberbotics.com/doc/guide/pioneer-3dx?version=master
    def __init__(self, tread=0.381, wheel_radius=0.195/2):
        super(MyController, self).__init__()
        self.timestep = int(self.getBasicTimeStep())

        hz = 50
        period = int(1000 / hz)
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

        self.kinect_color = self.getDevice("kinect color")
        self.kinect_color.enable(self.timestep)
        self.kinect_range = self.getDevice("kinect range")
        self.kinect_range.enable(self.timestep)
        self.lrf = self.getDevice("Hokuyo UTM-30LX")
        self.lrf.enable(self.timestep)

        self.ros_client = None
        self.connect_ros()
        self.pub_wheel_pos = Topic(
            self.ros_client, '/webots/wheel_pos', 'std_msgs/Float32MultiArray')
        self.pub_wheel_pos.advertise()
        self.pub_wheel_pos_timer = SimpleTimer(self.getTime(), 10.0)
        self.sub_wheel_vels = Topic(
            self.ros_client, '/webots/wheel_vels', 'std_msgs/Float32MultiArray')
        self.sub_wheel_vels.subscribe(self.wheel_vels_callback)
        self.wheel_vels = None
        self.wheel_vels_lock = Lock()

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
        if self.pub_wheel_pos_timer.check_elapsed(tm):
            pos = [self.left_wheel_sensor.getValue(),
                self.right_wheel_sensor.getValue()]
            self.pub_wheel_pos.publish(Message(build_ros_array_msg(pos)))

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
            if self.ros_client is not None and self.ros_client.is_connected:
                self.set_wheel_vels_from_rosmsg()
                self.publish_wheel_positions(tm)
            else:
                self.connect_ros()


# main Python program
controller = MyController()
controller.run()
