#!/home/luca/tiago_public_ws/src/vision_utils/pcd_venv/bin/python3

import os
import numpy as np
import open3d as o3d

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloud_ros_to_numpy(msg):
    """From: https://answers.ros.org/question/321829/color-problems-extracting-rgb-from-pointcloud2/

    Args:
        msg (_type_): _description_
    """
    pc = ros_numpy.numpify(msg)
    shape = pc.shape + (3, )    # add a dimension to store XYZ and RGB info
    
    points = np.zeros(shape) 
    points[..., 0] = pc['x']
    points[..., 1] = pc['y']
    points[..., 2] = pc['z']

    rgb = None
    if 'rgb' in pc.dtype.fields:
        # If the cloud has an RGB field, split it to obtain 3 int values from a single float value.
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)

        rgb = np.zeros(shape)
        rgb[..., 0] = pc['r']
        rgb[..., 1] = pc['g']
        rgb[..., 2] = pc['b']
    
    return points, rgb


def pointcloud_ros_to_open3d(msg):
    pts, rgb = pointcloud_ros_to_numpy(msg)
    pcd = o3d.geometry.PointCloud()
    # From Open3D documentation, the `points` attribute should be `float64 array of shape (num_points, 3)`. Hence, if the input cloud has shape (h, w, 3), it must be reshaped to 
    # TODO
    pcd.points = o3d.utility.Vector3dVector(pts)
    if rgb is not None:
        # From Open3D documentation, the `colors` attribute should be `float64 array of shape (num_points, 3)`. Hence, if the input cloud has shape (h, w, 3), it must be reshaped to 
        # TODO
        pcd.colors = o3d.utility.Vector3dVector(rgb)

def pointcloud_open3d_to_ros(msg):
    pass


if __name__ == "__main__":
    rospy.init_node('cloud_converter_test')
    #sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2,pointcloud_ros_to_open3d)
    
    msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    cloud = pointcloud_ros_to_open3d(msg)
    # Display the pointcloud to see if the conversion is correct

    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")