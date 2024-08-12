#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import utm
import rospy
import serial
import pynmea2
from math import pi, sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import ChannelFloat32
from tf import transformations, TransformBroadcaster
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped, Twist

class GpsNode(object):
    def __init__(self, serial_port, baud_rate, base_lat, base_lon):
        self.__x = float(0.0)
        self.__y = float(0.0)
        self.__yaw = float(0.0)
        self.__base_lat = base_lat
        self.__base_lon = base_lon
        self.__init_serial(serial_port, baud_rate)
        self.__dir = {'E': '东经', 'W': '西经', 'N': '北纬', 'S': '南纬'}
        self.__status = {0: '初始化', 1: '单点定位', 2: '码差分', 3: '无效GPS', 4: '固定解', 5: '浮点解'}
        self.__gps_broadcaster = TransformBroadcaster()
        self.__vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.__ros_pub_gps = rospy.Publisher('/gps_pose', PoseStamped, queue_size=1)
        self.__ros_pub_latlon = rospy.Publisher('/lat_lon', ChannelFloat32, queue_size=1)
        self.__fuse_odom = rospy.get_param("~fuse_odom", False)
        if self.__fuse_odom:
            odom_sub = rospy.Subscriber('/odom', Odometry, self.__odomCallback)
            self.__last_time = rospy.get_time()

    def __init_serial(self, serial_port, baud_rate):
        print("Init serial")
        try:
            self.__serial = serial.Serial(serial_port, baud_rate, timeout=1)
            print("connected to serial port")
        except serial.SerialException as e:
            sys.exit(e)

    def __odomCallback(self, odomData):
        vx = odomData.twist.twist.linear.x
        vy = odomData.twist.twist.linear.y
        vw = odomData.twist.twist.angular.z
        current_time = rospy.get_time()
        dt = current_time - self.__last_time
        dyaw = vw * dt
        dx = (vx * cos(self.__yaw) - vy * sin(self.__yaw)) * dt
        dy = (vx * sin(self.__yaw) + vy * cos(self.__yaw)) * dt
        self.__x += dx
        self.__y += dy
        self.__yaw += dyaw
        self.__last_time = current_time
        self.__pub_gps_pose(self.__x, self.__y, self.__yaw)

    def get_gps_data(self):
        lat = float(0.0)
        lon = float(0.0)
        bearing = float(0.0)
        got_gga = False
        got_hdt = False
        offset_x = rospy.get_param("~offset_x", float(0.2))
        offset_y = rospy.get_param("~offset_y", float(0.0))
        offset_yaw = rospy.get_param("~offset_yaw", float(0.0))
        base_point = utm.from_latlon(self.__base_lat, self.__base_lon)
        while not rospy.is_shutdown():
            try:
                data = self.__serial.readline().decode('utf-8')
                msg = pynmea2.parse(data)

                if "GGA" == msg.sentence_type and msg.lat:
                    rospy.loginfo_throttle(1, 'status: ' + self.__status[msg.gps_qual])
                    lat = msg.latitude
                    lon = msg.longitude
                    position = utm.from_latlon(lat, lon)
                    x = position[0] - base_point[0]
                    y = position[1] - base_point[1]
                    rospy.loginfo_throttle(1, (x, y))
                    '''蘑菇头位置补偿：GPS获取的坐标是以后面的蘑菇头为准，要把该坐标点移到车体旋转中心'''
                    self.__x = x + offset_x * cos(self.__yaw) - offset_y * sin(self.__yaw)
                    self.__y = y + offset_x * sin(self.__yaw) + offset_y * cos(self.__yaw)
                    _longitude = self.__dir[msg.lon_dir] + '%02d°%02d′%02d″, ' % (
                    msg.longitude, msg.longitude_minutes, msg.longitude_seconds)
                    _latitude = self.__dir[msg.lat_dir] + '%02d°%02d′%02d″' % (
                    msg.latitude, msg.latitude_minutes, msg.latitude_seconds)
                    rospy.loginfo_throttle(1, _longitude + _latitude)
                    got_gga = True
                    print("got GGA")

                if "HDT" == msg.sentence_type and msg.heading:
                    bearing = float(msg.heading)
                    # self.__yaw = (90-bearing+offset_yaw)/180.0*pi  #正东为0，X轴指东，y轴指北
                    self.__yaw = (48 + 360 - bearing + offset_yaw) / 180.0 * pi  # 正东为0，X轴指东，y轴指北
                    if self.__yaw > 2 * pi:
                        self.__yaw = self.__yaw - 2 * pi
                    rospy.loginfo_throttle(1, '方位角' + str(self.__yaw / pi * 180.0) + '度\n')
                    got_hdt = True
                    print("got HDT")

                if got_gga and got_hdt:
                    self.__pub_latlon(lat, lon, bearing)
                    if not self.__fuse_odom:
                        self.__pub_gps_pose(self.__x, self.__y, self.__yaw)

            except serial.SerialException as e:
                rospy.logerr('Serial error: {}'.format(e))
                break
            except pynmea2.ParseError as e:
                rospy.logerr('Parse error: {}'.format(e))
                continue
            except Exception as e:
                rospy.logerr("Unknown error: {}".format(e))

    def __pub_latlon(self, lat, lon, bearing):
        lat_lon = ChannelFloat32()
        lat_lon.name = "gps_latlon_data"
        lat_lon.values.append(lat)
        lat_lon.values.append(lon)
        lat_lon.values.append(bearing)
        self.__ros_pub_latlon.publish(lat_lon)

    def __pub_gps_pose(self, x, y, yaw):
        current_time = rospy.get_rostime()
        euler = transformations.quaternion_from_euler(0, 0, yaw)
        pose_quat = Quaternion(*euler)

        gps_pose = PoseStamped()
        gps_pose.header.frame_id = "gps"
        gps_pose.header.stamp = current_time
        gps_pose.pose.position.x = x
        gps_pose.pose.position.y = y
        gps_pose.pose.orientation = pose_quat
        self.__ros_pub_gps.publish(gps_pose)

        gps_trans = TransformStamped()
        gps_trans.header.frame_id = "gps"
        gps_trans.child_frame_id = "car_base_link"
        gps_trans.header.stamp = current_time
        gps_trans.transform.translation.x = x
        gps_trans.transform.translation.y = y
        gps_trans.transform.rotation = pose_quat
        self.__gps_broadcaster.sendTransformMessage(gps_trans)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.__vel_pub.publish(cmd_vel)
        rospy.loginfo("stop robot")


if __name__ == '__main__':
    rospy.init_node("gps_node", anonymous=True)
    latitude = rospy.get_param("~latitude", 31.7780360301891)
    longitude = rospy.get_param("~longitude", 117.2722178175)
    rospy.loginfo("base_lat = %.12f, base_lon = %.12f" % (latitude, longitude))

    try:
        gps = GpsNode("/dev/ttyUSB0", 115200, float(latitude), float(longitude))  # 修改为串口和波特率
        rospy.on_shutdown(gps.stop_robot)
        gps.get_gps_data()

    except Exception as message:
        rospy.logerr(message)

