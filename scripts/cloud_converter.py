#!/home/luca/tiago_public_ws/src/vision_utils/pcd_venv/bin/python3

"""This script implements a ROS publisher/subscriber node that acquires a ROS PointCloud2 message, converts it into an Open3D PointCloud object, and displays it. The node then converts back the Open3D PointCloud object into a ROS PointCloud2 message and broadcasts the message at 1Hz.
The above behavior is intended as a demo of the conversion tools implemented in the `vision_utils` package.
"""

import open3d as o3d

import rospy
from sensor_msgs.msg import PointCloud2

#from vision_utils.decoder import ...
from vision_utils.cloud_conversion import pointcloud_ros_to_open3d, pointcloud_open3d_to_ros

if __name__ == "__main__":
    rospy.init_node('cloud_converter_test')
    
    rate = rospy.Rate(1)   #[Hz]
    pub = rospy.Publisher('/open3d_to_ros/points', PointCloud2, queue_size = 1)
    
    msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    cloud, shape = pointcloud_ros_to_open3d(msg)
    print(shape)
    # Display the pointcloud to see if the conversion is correct
    o3d.visualization.draw_geometries([cloud], 
                                      lookat = [0, 0, 0],
                                      up = [0, -1, 0],
                                      front = [0, 0, -1],
                                      zoom = 0.3
                                      ) 
    
    try:
        while not rospy.is_shutdown():
            msg = pointcloud_open3d_to_ros(cloud, shape = shape)
            msg.header.frame_id = 'xtion_rgb_optical_frame'
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)    # check it in Rviz
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down the node.')