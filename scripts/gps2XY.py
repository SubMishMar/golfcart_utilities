#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import utm
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from vn300.msg import ins

class gpsUtilities:
    def __init__(self):

        self.gps_sub = rospy.Subscriber('/vectornav/ins', ins, self.gps_callback)
        self.pose_pub = rospy.Publisher('/Pose', PoseStamped, queue_size=10)
        self.firstRun = True
        self.global_home = np.array([0, 0, 0])
        self.global_position = np.array([0, 0, 0])
        self.local_positionNED = np.array([0, 0, 0])
        self.local_positionENU = np.array([0, 0, 0])
        self.br = tf.TransformBroadcaster()
        self.pose = PoseStamped()
        self.initial_bearing = 0
        self.bearing = 0
        self.init_rotn = np.identity(4)

    def gps_to_local(self):
        (easting_home, northing_home, zone_number_home, zone_letter_home) = utm.from_latlon(self.global_home[0], self.global_home[1])
        (easting, northing, zone_number, zone_letter) = utm.from_latlon(self.global_position[0], self.global_position[1])
        northing_local = northing - northing_home
        easting_local = easting - easting_home
        down_local = self.global_position[2] - self.global_home[2]
        self.local_positionNED = np.array([northing_local, easting_local, down_local])
        self.local_positionENU = np.array([northing_local, -easting_local, -down_local]);

    def gps_callback(self, data):
        self.global_position = np.array([data.LLA.x, data.LLA.y, data.LLA.z])

        if self.firstRun:
            self.global_home = self.global_position
            self.initial_bearing = -data.RPY.z*math.pi/180

        quatn_init = tf.transformations.quaternion_from_euler(0, 0, self.initial_bearing)
        transformation_init = tf.transformations.quaternion_matrix(quatn_init)

        self.gps_to_local()
        self.local_positionENU[2] = 0
        transformation_ = tf.transformations.translation_matrix(self.local_positionENU)
        self.bearing = (-data.RPY.z) * math.pi/180; 
        quatn_ = tf.transformations.quaternion_from_euler(0, 0, self.bearing)
        rotation_ = tf.transformations.quaternion_matrix(quatn_ )
        transformation_current = np.dot(transformation_, rotation_)
        transformation_current = np.dot(np.linalg.inv(transformation_init), transformation_current)
        trans = tf.transformations.translation_from_matrix(transformation_current)
        quatn = tf.transformations.quaternion_from_matrix(transformation_current)

        self.br.sendTransform((trans[0], trans[1], trans[2]), quatn, data.header.stamp, "base_link", "map")

        self.pose.header.stamp = data.header.stamp
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = trans[0] #+ sn*self.local_positionENU[1]
        self.pose.pose.position.y = trans[1] #+ cs*self.local_positionENU[1]
        self.pose.pose.position.z = trans[2]
        self.pose.pose.orientation.x = quatn[0]
        self.pose.pose.orientation.y = quatn[1]
        self.pose.pose.orientation.z = quatn[2]
        self.pose.pose.orientation.w = quatn[3]
        self.pose_pub.publish(self.pose)

        self.firstRun = False
        
if __name__ == '__main__':
    rospy.init_node('gps2XY', anonymous=True)
    gpsUtls = gpsUtilities()
    rospy.spin()