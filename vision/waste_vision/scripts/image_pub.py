#!/usr/bin/env python3
import rospy
from waste_vision.msg import CroppedObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from pathlib import Path

class TestCroppedObjectPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_publisher', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publisher for String messages
        self.pub = rospy.Publisher('/image_topic', String, queue_size=10)
        
        # Get the script's directory
        self.script_dir = Path(__file__).parent
        
        # Construct relative path to the test image
        self.test_image_path = self.script_dir / "../src/part1_complete/multi_test.jpg"
        
        # Check if the image file exists
        if not self.test_image_path.exists():
            rospy.logerr(f"Test image not found: {self.test_image_path}")
            raise FileNotFoundError(f"Test image not found: {self.test_image_path}")
        
        print(f"Using test image: {self.test_image_path}")
        
        # Publish rate (Hz)
        self.rate = rospy.Rate(0.1)  # 0.1 Hz (every 10 seconds)
        
        rospy.loginfo("Test CroppedObject Publisher Initialized")
    
    def publish(self):
        while not rospy.is_shutdown():
            try:
                # Publish the image path as a String message
                msg = str(self.test_image_path)
                self.pub.publish(msg)
                rospy.loginfo('Image path published.')
                
            except Exception as e:
                rospy.logerr(f"Error publishing message: {e}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestCroppedObjectPublisher()
        publisher.publish()
    except rospy.ROSInterruptException:
        pass