#!/usr/bin/env python

import message_filters
import rospy

from geometry_msgs.msg import PointStamped, PolygonStamped
from stamped_msgs.msg import Float32
from visualization_msgs.msg import Marker

class RobotRadiusVisualization(object):
    """
    A simple class that combines robot_radius and robot_center topics into rviz markers
    """

    def __init__(self):

        sub_radius = message_filters.Subscriber("robot_radius", Float32)
        sub_center = message_filters.Subscriber("robot_center", PointStamped)
        self.sub_aabb = rospy.Subscriber("robot_bounding_box", PolygonStamped, self.aabb_callback, queue_size=10)

        self.ts = message_filters.TimeSynchronizer([sub_radius, sub_center], 10)
        self.ts.registerCallback(self.radius_callback)

        self.radius_pub = rospy.Publisher("robot_radius_visualization", Marker, queue_size=10)
        self.aabb_pub = rospy.Publisher("robot_bounding_box_visualization", Marker, queue_size=10)

    def radius_callback(self, radius_msg, center_msg):
        marker = Marker()
        marker.header = radius_msg.header
        marker.ns = "robot_radius_visualization"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = center_msg.point.x
        marker.pose.position.y = center_msg.point.y
        marker.pose.position.z = center_msg.point.z
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = 2 * radius_msg.data
        marker.color.a = 0.5
        marker.frame_locked = True

        self.radius_pub.publish(marker)

    def aabb_callback(self, polygon_msg):
        assert isinstance(polygon_msg, PolygonStamped)
        marker = Marker()
        marker.header = polygon_msg.header
        marker.ns = "robot_bounding_box_visualization"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.color.a = 0.5
        marker.scale.x = polygon_msg.polygon.points[1].x - polygon_msg.polygon.points[0].x
        marker.scale.y = polygon_msg.polygon.points[1].y - polygon_msg.polygon.points[0].y
        marker.scale.z = polygon_msg.polygon.points[1].z - polygon_msg.polygon.points[0].z
        marker.pose.position.x = marker.scale.x/2 + polygon_msg.polygon.points[0].x
        marker.pose.position.y = marker.scale.y/2 + polygon_msg.polygon.points[0].y
        marker.pose.position.z = marker.scale.z/2 + polygon_msg.polygon.points[0].z
        marker.pose.orientation.w = 1.
        marker.frame_locked = True

        self.aabb_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('robot_radius_visualization')

    cls = RobotRadiusVisualization()

    rospy.spin()
