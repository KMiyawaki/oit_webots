import math
import time

class LidarRos(object):
    def __init__(self, lidar, frame_id="base_laser_link"):
        self.lidar = lidar
        self.angle_min = self.lidar.getFov() / 2.0
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
        unix_time = time.time() # for ROS
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
            'ranges': self.lidar.getRangeImage(),
            'intensities': [],    # intensity data [device-specific units
        }
        self.seq = self.seq + 1
        return msg


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
            return True
        return False


def build_ros_header(webots_time, frame_id, seq):
    s2, s = math.modf(webots_time)
    return {
        'stamp': {'secs': int(s), 'nsecs': int(s2 * 1000000000)},
        'frame_id': frame_id,
        'seq': seq
    }


def build_ros_array_msg(data):
    return {'data': data,
            'layout': {
                'dim': [{'stride': 0, 'size': 0, 'label': ''}],
                'data_offset': 0}
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
            self.x += radius * (math.sin(new_yaw) - math.sin(self.yaw))
            self.y -= radius * (math.cos(new_yaw) - math.cos(self.yaw))
            self.yaw = normalize_angle(new_yaw)
        else:
            d_yaw = 0
            self.x += d_total * math.cos(self.yaw)
            self.y += d_total * math.sin(self.yaw)

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
