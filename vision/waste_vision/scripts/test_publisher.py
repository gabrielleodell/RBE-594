#!/usr/bin/env python3
import rospy
from waste_vision.msg import CroppedObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TestCroppedObjectPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('test_cropped_object_publisher', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publisher for CroppedObject messages
        self.pub = rospy.Publisher('/cropped_object', CroppedObject, queue_size=10)
        
        # Load a test image (update path or use dummy image)
        self.test_image_path = '/home/nddixon/catkin_ws/src/waste_vision/test_images/glass.jpg'  # Update with actual path
        self.test_image = self.load_test_image()
        
        # Sample coordinates (adjust as needed)
        self.x = 100.0
        self.y = 200.0
        
        # Publish rate (Hz)
        self.rate = rospy.Rate(1)  # 1 Hz
        
        rospy.loginfo("Test CroppedObject Publisher Initialized")
    
    def load_test_image(self):
        try:
            # Load image from file
            img = cv2.imread(self.test_image_path)
            if img is None:
                raise ValueError("Failed to load image")
            rospy.loginfo(f"Loaded test image from {self.test_image_path}")
            return img
        except Exception as e:
            # Fallback to dummy image
            rospy.logwarn(f"Error loading image: {e}. Using dummy image.")
            return np.zeros((100, 100, 3), dtype=np.uint8)  # 100x100 black image
    
    def publish(self):
        while not rospy.is_shutdown():
            try:
                # Create CroppedObject message
                msg = CroppedObject()
                msg.image = self.bridge.cv2_to_imgmsg(self.test_image, encoding='bgr8')
                msg.x = self.x
                msg.y = self.y
                
                # Publish message
                self.pub.publish(msg)
                rospy.loginfo(f"Published CroppedObject: x={msg.x}, y={msg.y}, image_shape={self.test_image.shape}")
                
            except Exception as e:
                rospy.logerr(f"Error publishing message: {e}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestCroppedObjectPublisher()
        publisher.publish()
    except rospy.ROSInterruptException:
        pass