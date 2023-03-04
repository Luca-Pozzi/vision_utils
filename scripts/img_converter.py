#!/usr/bin/env python

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, CompressedImage
from vision_utils import image_conversion

class DecoderTest:
    def __init__(self, sim = True):
        # Get ready to receive images
        self.img_rgb        = None
        self.img_depth      = None
        self.img_rgb_cmpr   = None
        self.img_depth_cmpr = None
        # Get ready to draw the decoded images
        self.img_display    = None
        
        # Subscribe to Image and CompressedImage topics
        rospy.Subscriber('/xtion/rgb/image_raw', 
                         Image, 
                         self.rgb_cb)
        rospy.Subscriber('/xtion/depth_registered/image_raw', 
                         Image, 
                         self.depth_cb)
        rospy.Subscriber('/xtion/rgb/image_raw/compressed', 
                         CompressedImage, 
                         self.rgb_cmpr_cb)
        rospy.Subscriber('/xtion/depth_registered/image_raw/compressed', 
                         CompressedImage, 
                         self.depth_cmpr_cb)
        # NOTE. There are no compressedDepth topics available in simulation.

    def rgb_cb(self, msg):
        img = image_conversion.decode_Image_RGB(msg)
        self.img_rgb = img

    def depth_cb(self, msg):
        img = image_conversion.decode_Image_depth(msg)
        self.img_depth = img
    
    def rgb_cmpr_cb(self, msg):
        img = image_conversion.decode_CompressedImage_RGB(msg)
        self.img_rgb_cmpr = img
    
    def depth_cmpr_cb(self, msg):
        img = image_conversion.decode_CompressedImage_depth(msg)
        self.img_depth_cmpr = img

    def draw(self):
        # The RGB Image message is arbitrarily chosen to be the one to wait before start plotting
        if self.img_display is None and self.img_rgb is not None:
            h, w, c = self.img_rgb.shape    
            self.img_display = np.zeros((2 * h, 2 * w, c), dtype = float)
        
        # Set the RGB Image in the top left corner
        self.img_display[:h, :w, :] = self.img_rgb[:] / 255.0
        # Set the depth Image in the top right corner
        if self.img_depth is not None:
            self.img_display[:h, w:, :] = np.repeat(self.img_depth[..., np.newaxis], 
                                                    3,       
                                                    axis=-1) / np.nanmax(self.img_depth)
        # Set the RGB CompressedImage in the bottom left corner
        if self.img_rgb_cmpr is not None:
            self.img_display[h:, :w, :] = self.img_rgb_cmpr[:] / 255.0 
        # Set the depth CompressedImage in the bottom right corner
        if self.img_depth_cmpr is not None:
            self.img_display[h:, w:, :] = np.repeat(self.img_depth_cmpr[..., np.newaxis], 
                                                    3,       
                                                    axis=-1) / np.nanmax(self.img_depth_cmpr)
        # TODO. Could be made fancier, writing topic names and displaying `Not available` if corresponding images are not received.

if __name__ == '__main__':
    rospy.init_node('test')

    d = DecoderTest()

    try:
        while not rospy.is_shutdown():
            d.draw()
            if d.img_display is not None:
                cv2.imshow('RGB and depth data from ROS topics', d.img_display.copy())
                cv2.waitKey(1)
    except KeyboardInterrupt:
        pass