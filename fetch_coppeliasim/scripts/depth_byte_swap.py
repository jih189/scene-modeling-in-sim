#! /usr/bin/python

import numpy as np
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np

class ByteSwapDepthImage:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = "/head_camera/depth/image_raw"
        self.out_topic = "/head_camera/depth/image_corrected"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher(self.out_topic, Image, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2_img.byteswap(True)
            r = self.bridge.cv2_to_imgmsg(cv2_img, "passthrough")
            # copy timestamp and frame_id
            r.header = msg.header

            self.image_pub.publish(r)

        except CvBridgeError, e:
            print(e)


if __name__ == '__main__':
    # main()
    rospy.init_node('byte_swap_depth_image', anonymous=True)
    ByteSwapDepthImage = ByteSwapDepthImage()
    while not rospy.is_shutdown():
        rospy.spin()
