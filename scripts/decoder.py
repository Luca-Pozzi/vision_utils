import os
import cv2
import struct
import numpy as np

import rospy
from sensor_msgs.msg import Image, CompressedImage

'''
NOTE: requires OpenCV >= 3.0 (as the one I have locally installed is), with older versions, cv2.IMREAD_UNCHANGED and cv2.IMREAD_COLOR should be adapted
'''

def check_for_type(topic, timeout=1):
    """Check wheter a topic is publishing Image or CompressedImage messages.

    Args:
        topic (string): the name of the topic to check.
        timeout (int, optional): the time interval [s] for which topic is listened, waiting for a message. Defaults to 1.

    Returns:
        - <class 'sensor_msgs.msg._Image.Image'> or <class 'sensor_msgs.msg._CompressedImage.CompressedImage'>:
          the type of the messages received on topic. Defaults to None if no message is received within the timeout.
        - string:
          the enconding type of the image message
    """
    try:    # assume the topic is publishing Image messages
        encoding = rospy.wait_for_message(
            topic, Image, timeout=timeout).encoding
        msg_type = Image
    except rospy.ROSException:
        pass  # fallback to CompressedImage
    try:    # check if the topic is publishing CompressedImage messages instead
        encoding, __ = rospy.wait_for_message(topic,
                                              CompressedImage,
                                              timeout=1
                                              ).format.split(';')
        encoding = encoding.strip()
        msg_type = CompressedImage
    except rospy.ROSException:
        msg_type = None
        encoding = None
        rospy.logerr(
            'No Image or CompressedImage can be received on topic', topic)
    '''
    NOTE: it could be a good idea to cast msg_type to a string, so to have a unique return type.
    '''
    return msg_type, encoding


def decode_CompressedImage_depth(msg, header_size=12, depth_in_m=True):
    """Converts a CompressedImage ROS message encoding depth information into a numpy ndarray.
    From https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/

    Args:
        msg (<class 'sensor_msgs.msg._CompressedImage.CompressedImage'>): the CompressedImage message to be converted into numpy
        header_size (int, optional): the length of the header in byte. Defaults to 12.
        depth_in_m (bool, optional): if True, the depth values are expressed in m, otherwise in mm. Defaults to True.

    Raises:
        Exception: the compression type is not recognized.
        Exception: the header is not recognized, maybe because of an incorrect header_size parameter.
        Exception: the format of the depth data is unknown.

    Returns:
        - numpy.ndarray:
          depth data. Dimensions are HxW where H and W are the image height and width [px] respectively. Data should be interpreted as values in m or mm depending on the depth_in_m value.
        - string:
          either "16UC1" (np.uint16) or "32FC1" (np.float32)
    """
    # 'msg' as type CompressedImage
    depth_fmt, compr_type = msg.format.split(';')
    # Remove white space
    depth_fmt = depth_fmt.strip()
    compr_type = compr_type.strip()
    if compr_type != "compressedDepth":
        raise Exception("Compression type is not 'compressedDepth'."
                        "You probably subscribed to the wrong topic.")

    # Remove header from raw data
    raw_data = msg.data[header_size:]

    # TODO: try frombuffer instead of fromstring
    depth_img_raw = cv2.imdecode(np.fromstring(
        raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
    if depth_img_raw is None:   # probably wrong header
        raise Exception("Could not decode compressed depth image."
                        "You may need to change 'header_size'.")

    if depth_fmt == "16UC1":    # data in mm as uint16
        # Convert data to m if requested by the user (dividing should automatically turn depth data
        # into float)
        depth_data = depth_img_raw / 1000 if depth_in_m else depth_img_raw
    elif depth_fmt == "32FC1":
        raw_header = msg.data[:header_size]
        # header: int, float, float
        [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
        depth_data = depthQuantA / \
            (depth_img_raw.astype(np.float32)-depthQuantB)
        # filter max values
        depth_data[depth_img_raw == 0] = 0

        # depth_data provides distance in meters as f32, convert to mm if request by the user
        depth_data = depth_data if depth_in_m else (
            depth_data * 1000).astype(np.uint16)
        # reshape needed here?
    else:
        raise Exception("Decoding of '" + depth_fmt + "' is not implemented!")

    # Data are uint16 if in mm, in float32 if in m. Also the encoding is returned.
    return depth_data, depth_fmt


def decode_Image_depth(msg, depth_in_m=True):
    """Converts a Image ROS message encoding depth information into a numpy ndarray.

    Args:
        msg (<class 'sensor_msgs.mgs._Image.Image'>): the Image message to be converted into numpy.
        depth_in_m (bool, optional): if True, the depth values are expressed in m, otherwise in mm. Defaults to True.

    Returns:
        - numpy.ndarray:
          depth data. Dimensions are HxW where H and W are the image height and width [px] respectively. Data should be interpreted as values in m or mm depending on the depth_in_m value.
        - string:
          either "16UC1" (np.uint16) or "32FC1" (np.float32)
    """
    if msg.encoding == "16UC1":
        byte_array = np.fromstring(
            ros_depth.data, np.uint16)       # array of bytes
        depth_data = np.frombuffer(
            byte_array, dtype=np.uint16)     # numpy 2D array
        depth_data = depth_data / 1000 if depth_in_m else depth_data
    elif msg.encoding == "32FC1":
        byte_array = np.fromstring(
            ros_depth.data, np.float32)      # array of bytes
        depth_data = np.frombuffer(
            byte_array, dtype=np.float32)    # numpy 2D array
        depth_data = depth_data if depth_in_m else (
            depth_data * 1000).astype(np.uint16)

    depth_data = np.reshape(depth_data,
                            newshape=(msg.height, msg.width)
                            )
    # Data are uint16 if in mm, in float32 if in m. Also the encoding is returned.
    return depth_data, depth_fmt


def decode_Image_RGB(msg):
    """Convert a ROS Image message into an OpenCV image.

    Args:
        msg (<class 'sensor_msgs.msg._Image.Image'>): the ROS Image message to be converted into an OpenCV image.

    Returns:
        image array: the OpenCV image.
    """
    byte_array = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(byte_array, cv2.IMREAD_COLOR)
    return image


def decode_CompressedImage_RGB(msg):
    """Convert a ROS Image message into an OpenCV image.

    Args:
        msg (<class 'sensor_msgs.msg._Image.Image'>): the ROS Image message to be converted into an OpenCV image.

    Returns:
        image array: the OpenCV image.
    """
    byte_array = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(byte_array, cv2.IMREAD_COLOR)
    return image
