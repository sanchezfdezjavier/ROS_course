#!/usr/bin/env python

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2

def callback_print_points(msg):
    rospy.loginfo("Message received")
    #xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    record_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    pc2_to_record_to_pc2 = ros_numpy.point_cloud2.array_to_pointcloud2(record_array)
    #bag_publisher.publish(ros_numpy.point_cloud2.array_to_pointcloud2(xyz_array))
    #rospy.loginfo(record_array)
    rospy.loginfo(pc2_to_record_to_pc2)
    #rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node('lidar')

    bag_subscriber = rospy.Subscriber("/velodyne_points", PointCloud2, callback_print_points)
    #bag_publisher = rospy.Publisher("/filtered_points", PointCloud2, queue_size=1)

    rate = rospy.Rate(10)
    rospy.spin()



