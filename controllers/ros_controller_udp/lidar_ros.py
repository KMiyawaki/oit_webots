# -*- coding: utf_8 -*-

import math
import time
from utils import build_ros_header


class LidarRos(object):
    def __init__(self, lidar, frame_id="base_laser_link"):
        self.lidar = lidar
        self.angle_min = -self.lidar.getFov() / 2.0
        self.angle_max = self.lidar.getFov() / 2.0
        self.angle_increment = self.lidar.getFov() / self.lidar.getHorizontalResolution()
        self.scan_time = self.lidar.getSamplingPeriod() / 1000.0
        self.time_increment = self.scan_time / self.lidar.getHorizontalResolution()
        self.range_min = self.lidar.getMinRange()
        self.range_max = self.lidar.getMaxRange()
        self.frame_id = frame_id
        self.seq = 0

    def __str__(self):
        return "angle: %f <> %f, angle_increment: %f, scan_time: %f, time_increment: %f, range: %f <> %f" % (math.degrees(self.angle_min), math.degrees(self.angle_max), math.degrees(self.angle_increment),
                                                                                                             self.scan_time, self.time_increment, self.range_min, self.range_max)

    def get_ros_msg(self):
        unix_time = time.time()  # for ROS
        scan_org = list(reversed(self.lidar.getRangeImage()))
        scan = []
        for s in scan_org:
            if s == float('inf'):
                scan.append(0)
            else:                
                scan.append(round(s, 2))

        msg = {
            'header': build_ros_header(unix_time, self.frame_id, self.seq),
            'angle_min': self.angle_min,        # start angle of the scan [rad]
            'angle_max': self.angle_max,        # end angle of the scan [rad]
            # angular distance between measurements [rad]
            'angle_increment': self.angle_increment,
            # time between measurements [seconds] - if your scanner
            # is moving, this will be used in interpolating position
            # of 3d points
            'time_increment': self.time_increment,
            'scan_time': self.scan_time,        # time between scans [seconds]

            'range_min': self.range_min,        # minimum range value [m]
            'range_max': self.range_max,        # maximum range value [m]

            # range data [m] (Note: values < range_min or > range_max should be discarded)
            'ranges': scan,
            'intensities': [],    # intensity data [device-specific units
        }
        self.seq = self.seq + 1
        return msg
