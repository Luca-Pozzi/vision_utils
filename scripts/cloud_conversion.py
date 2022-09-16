#!/home/luca/tiago_public_ws/src/vision_utils/pcd_venv/bin/python3

import os
import numpy as np
import open3d as o3d

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloud_ros_to_open3d(msg):
    """From: https://answers.ros.org/question/321829/color-problems-extracting-rgb-from-pointcloud2/

    Args:
        msg (_type_): _description_
    """
    pc = ros_numpy.numpify(msg)
    pc = ros_numpy.point_cloud2.split_rgb_field(pc)
    
    shape = pc.shape + (3, )

    points = np.zeros(shape) 
    points[..., 0] = pc['x']
    points[..., 1] = pc['y']
    points[..., 2] = pc['z']
    rgb = np.zeros(shape)
    rgb[..., 0] = pc['r']
    rgb[..., 1] = pc['g']
    rgb[..., 2] = pc['b']


def pointcloud_open3d_to_ros(msg):
    pass

# TODO: to numpy?

if __name__ == "__main__":
    rospy.init_node('cloud_converter_test')
    sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, pointcloud_ros_to_open3d)

    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")