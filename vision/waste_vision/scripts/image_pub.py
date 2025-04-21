#!/usr/bin/env python3

import os
import cv2
import rospy
import numpy as np

from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from waste_vision.msg import CroppedObject

class TestCroppedObjectPublisher:
    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('image_publisher', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publisher for CroppedObject messages
        self.pub = rospy.Publisher('/image_topic', String, queue_size=10)

        # Subscribe to robot signals
        self.sub = rospy.Subscriber('/cycle_start', Bool, self.callback)
        
        # Get the script's directory
        self.script_dir = Path(__file__).parent.parent
        
        # Construct relative path to the test image
        self.image_folder = self.script_dir / "test_images/multi_test"

        # Pull next timage
        self.images = os.listdir(self.image_folder)
        for idx in range(len(self.images)):
            self.images[idx] = os.path.join(self.image_folder, self.images[idx])
        
        self.image_count = 0

        rospy.loginfo("Test CroppedObject Publisher Initialized")
        rospy.loginfo("Waiting for ready signal...")

    
    def publish(self, img_path):
        try:
            # Publish the image path as a String message
            self.pub.publish(str(img_path))
            rospy.loginfo('Image path published.')

            self.image_count += 1

            rospy.loginfo("Waiting for ready signal...")

        except Exception as e:
            rospy.logerr(f"Error publishing message: {e}")


    def callback(self, msg):
        ready_status = msg.data
        if ready_status:
            if self.image_count < len(self.images):
                self.publish(self.images[self.image_count])
            else:
                rospy.logwarn("Out of images.")
                return


if __name__ == '__main__':
    try:
        publisher = TestCroppedObjectPublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass