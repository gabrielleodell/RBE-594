#!/usr/bin/env python3
import rospy
from waste_vision.msg import CroppedObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

class TestCroppedObjectPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_publisher', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publisher for CroppedObject messages
        self.pub = rospy.Publisher('/image_topic', String, queue_size=10)
        
        # Load a test image (update path or use dummy image)
        self.test_image_path = '/home/nddixon/catkin_ws/src/waste_vision/src/part1_complete/multi_test.jpg'  # Update with actual path

        # Publish rate (Hz)
        self.rate = rospy.Rate(0.1)  
        
        rospy.loginfo("Test CroppedObject Publisher Initialized")
    

    
    def publish(self):
        while not rospy.is_shutdown():
            try:
                # Create CroppedObject message
                msg = self.test_image_path

                # Publish message
                self.pub.publish(msg)
                rospy.loginfo('Image published.')
                
            except Exception as e:
                rospy.logerr(f"Error publishing message: {e}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestCroppedObjectPublisher()
        publisher.publish()
    except rospy.ROSInterruptException:
        pass