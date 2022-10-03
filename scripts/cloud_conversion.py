#!/home/luca/tiago_public_ws/src/vision_utils/pcd_venv/bin/python3

import numpy as np
import open3d as o3d

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2

def pointcloud_ros_to_numpy(msg):
    """Convert a ROS PointCloud2 message into two np.ndarrays (one for XYZ coordinates, and one for RGB values).
    From: https://answers.ros.org/question/321829/color-problems-extracting-rgb-from-pointcloud2/

    Args:
        msg (sensor_msgs/PointCloud2): A ROS PointCloud2 message.

    Returns:
        A tuple containing a first np.ndarray (XYZ coordinates of the points in the cloud), a second nd.array (RGB values), and a tuple (shape of the input cloud). The second item (RGB values) can be None if the input PointCloud2 has no color information. 
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
    
    return points, rgb, shape[:-1]


def format_ndarray_for_o3d(arr):
    """Check if the input array is ordered (i.e. points are organized in a 2D matrix) or not. Unordered clouds are left untouched, ordered ones are flattened in order to match the (n, 3) shape required from Open3D PointCloud fields.

    Args:
        cloud (np.ndarray): The cloud to check and, possibly, flatten.

    Raises:
        ValueError: The shape of the input array is not (n, 3) or (h, w, 3).

    Returns:
        np.ndarray: The input array, reshaped to match (n, 3) shape.
    """
    # Remove any axes of lenght 1 (i.e. dummy dimensions).
    arr = np.squeeze(arr)
    # Check if cloud shape is suitable for the intended use
    if (not arr.shape[-1] == 3) or (not len(arr.shape) <= 3):
        raise ValueError('The input array is expected to be a numpy array with shape (n, 3) or (h, w, 3). After squeezing, the input array has shape ' + str(arr.shape))
    
    # Reshape the array as (n, 3). If the input array already has shape (n, 3), it remains unchanged.
    arr = np.reshape(arr, newshape = (-1, 3))

    return arr

def pointcloud_ros_to_open3d(msg):
    """Convert a ROS PointCloud2 message into a Open3D PointCloud object.

    Args:
        msg (sensor_msgs/PointCloud2): A ROS PointCloud2 message.

    Returns:
        open3d.geometry.PointCloud: An Open3D PointCloud object.
    """
    pts, rgb, ros_cloud_shape = pointcloud_ros_to_numpy(msg)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(format_ndarray_for_o3d(pts))
    if rgb is not None:
        rgb = (rgb - rgb.min()) / (rgb.max() - rgb.min()) # normalize RGB values in the [0 1] range
        pcd.colors = o3d.utility.Vector3dVector(format_ndarray_for_o3d(rgb))

    return pcd, ros_cloud_shape

def pointcloud_open3d_to_ros(cloud, shape = None, merge_rgb = True):
    """Convert an Open3D PointCloud object into a ROS PointCloud2 message.

    Args:
        cloud (open3d.geometry.PointCloud): A Open3D PointCloud object. 
        shape (tuple, optional): If specified should be a two-items (`h x w`) tuple with the 2D structure of the output pointcloud (i.e. points will be organized in a `h x w` matrix). Defaults to None.
        merge_rgb (bool, optional): If `True`, . Defaults to `True`.

    Returns:
        sensor_msgs/PointCloud2: A ROS message with the same data of the input Open3D cloud.
    """
    xyz = np.asarray(cloud.points)
    if shape is not None:
        h, w = shape    # unpack the shape tuple
        if not h * w == xyz.shape[0]:
            raise ValueError('Specified shape argument ' + str(shape) + ' would give ' + str(h * w) + 'points. This value does not match the number of points in the cloud ' + str(xyz.shape[0]))
        structured = True
    else:
        shape = xyz.shape[0]
        structured = False

    if cloud.colors: 
        # If the input cloud has color information, format it to be saved in the ROS cloud
        rgb = np.asarray(cloud.colors)
        rgb = (rgb * 255.0).astype(np.uint32)
    else:
        # If the input cloud does not have color info, paint it black.
        # NOTE: this is a workaround to keep the same structure for the `data` array below (i.e. to always have `r`, `g`, and `b` fields).
        rgb = np.zeros((xyz.shape[0], 3), dtype = np.uint32)
    
    if structured:
        xyz = np.reshape(xyz, newshape = shape + (3,))
        rgb = np.reshape(rgb, newshape = shape + (3,))
    
    # Create a structured array
    dtypes = [('x',    np.float32),
              ('y',    np.float32),
              ('z',    np.float32),
            ]
    dtype_rgb = [('rgb', np.float32)] if merge_rgb else [('r',    np.uint8),
                                                         ('g',    np.uint8),
                                                         ('b',    np.uint8)
                                                        ]
    dtypes.extend(dtype_rgb)
    # Fill the structured array with data
    data = np.zeros(shape,             # 1D or 2D 
                    dtype = dtypes
                    )
    # Fill in the fields with the point coordinates
    data['x'] = xyz[..., 0]
    data['y'] = xyz[..., 1]
    data['z'] = xyz[..., 2]
    # Fill in the fields with the RGB information.
    r = rgb[..., 0]
    g = rgb[..., 1]
    b = rgb[..., 2]
    if merge_rgb:
        '''
        NOTE: the following two lines are taken from `ros_numpy.point_cloud2.merge_rgb_fields`
        '''
        rgb = np.array((r << 16) | (g << 8) | (b << 0), dtype = np.uint32)
        rgb.dtype = np.float32
        data['rgb'] = rgb
    else:
        data['r'] = r.astype(np.uint8) 
        data['g'] = g.astype(np.uint8)
        data['b'] = b.astype(np.uint8)
    
    return ros_numpy.msgify(PointCloud2, data)


if __name__ == "__main__":
    rospy.init_node('cloud_converter_test')
    
    rate = rospy.Rate(1)   #[Hz]
    pub = rospy.Publisher('/open3d_to_ros/points', PointCloud2, queue_size = 1)
    
    msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    cloud, shape = pointcloud_ros_to_open3d(msg)
    '''
    # Display the pointcloud to see if the conversion is correct
    o3d.visualization.draw_geometries([cloud], 
                                      lookat = [0, 0, 0],
                                      up = [0, -1, 0],
                                      front = [0, 0, -1],
                                      zoom = 0.3
                                      ) 
    '''
    try:
        while not rospy.is_shutdown():
            msg = pointcloud_open3d_to_ros(cloud, shape = shape)
            msg.header.frame_id = 'xtion_rgb_optical_frame'
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down the node.')