#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from waste_vision.msg import CroppedObject, ClassifiedObject
from cv_bridge import CvBridge
import torch
from torch import nn
import torchvision
import torchvision.transforms as transforms
from torchvision.models import resnet18
import json
import os

class WasteClassifierNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waste_classifier_node', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Device setup
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load class mappings
        self.class_labels = self.load_class_mappings()
        
        # Load the deep learning model
        self.res4_model = self.load_res4_model()
        
        # Image preprocessing transforms (matching training)
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        # Subscriber for cropped object (image + coordinates)
        self.sub = rospy.Subscriber('/cropped_object', CroppedObject, self.object_callback)
        
        # Publisher for classified object (class + coordinates)
        self.pub = rospy.Publisher('/classified_objects', ClassifiedObject, queue_size=10)
        
        rospy.loginfo("Waste Classifier Node Initialized")
    
    def load_class_mappings(self):
        # Load class mappings from JSON
        class_mapping_path = '/home/nddixon/catkin_ws/src/waste_vision/models/resnet/resnet_4/class_mapping.json'  # Update path
        try:
            with open(class_mapping_path, 'r') as f:
                class_mapping = json.load(f)
            # Convert indices (stored as strings in JSON) to integers and sort by index
            class_labels = [class_mapping[str(i)] for i in range(len(class_mapping))]
            rospy.loginfo(f"Loaded class labels: {class_labels}")
            return class_labels
        except Exception as e:
            rospy.logerr(f"Failed to load class mappings: {e}")
            # return ['plastic', 'metal', 'paper', 'glass', 'organic']  # Fallback
    
    def load_res4_model(self):
        try:
            checkpoint = torch.load('/home/nddixon/catkin_ws/src/waste_vision/models/resnet/resnet_4/res_best.pth', map_location=torch.device(self.device))# Extract the class mapping from the checkpoint
            class_mapping = checkpoint.get('class_mapping', {})
            
                # Convert keys to string to handle potential integer/string key differences
            class_names = [class_mapping[i] for i in range(len(class_mapping))]# Create a reverse mapping from class name to index
            label_to_idx = {name: i for i, name in enumerate(class_names)}
            print(f"Classes: {class_names}")# Create model with the right number of classes
            model = torchvision.models.resnet18()
            num_ftrs = model.fc.in_features
            model.fc = nn.Linear(num_ftrs, len(class_names))# Load model weights from checkpoint
            model.load_state_dict(checkpoint['model_state_dict'])
            model.to(self.device)
            model.eval()
            return model
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            return None
    
    def preprocess_image(self, cv_image):
        # Convert BGR (OpenCV) to RGB
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Apply PyTorch transforms
        img = self.transform(img)
        # Add batch dimension
        img = img.unsqueeze(0)
        return img
    
    def object_callback(self, msg):
        try:
            # Extract image and coordinates from the custom message
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
            x, y = msg.x, msg.y
            
            # Preprocess the image for the model
            processed_image = self.preprocess_image(cv_image).to(self.device)
            
            # Run inference
            if self.res4_model is None:
                rospy.logerr("Model not loaded")

                return
            
            with torch.no_grad():  # Disable gradient computation for inference
                outputs = self.res4_model(processed_image)
                _, predicted_class_idx = torch.max(outputs, 1)
                predicted_class = self.class_labels[predicted_class_idx.item()]
            
            # Publish the result
            classified_msg = ClassifiedObject()
            classified_msg.class_label = predicted_class
            classified_msg.x = x
            classified_msg.y = y
            
            self.pub.publish(classified_msg)
            rospy.loginfo(f"Published: Class={predicted_class}, x={x}, y={y}")
                
        except Exception as e:
            rospy.logerr(f"Error processing object: {e}")
    
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = WasteClassifierNode()
        node.run()
    except rospy.ROSInterruptException:
        pass