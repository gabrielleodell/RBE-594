#!/usr/bin/env python3
import rospy
import cv2
from waste_vision.msg import PointArray, ClassifiedPoint
import torch
from torch import nn
import torchvision
import torchvision.transforms as transforms
from torchvision.models import resnet18, ConvNeXt_Small_Weights
import json
import os
from ultralytics import YOLO
from std_msgs.msg import String
from pathlib import Path

from yolo_crop import my_crop_with_loaded_model
from generate_pick_path_with_classes import simulated_annealing, bins_px, start_position_px, return_path_with_class
from convert_labels_to_world_centers import labels_to_world
from detect_and_crop_onnx import ONNXDetector, my_crop_with_onnx

class VisionSystemNode:
    def __init__(self, classes: int, classifier: str, detector: str):
        rospy.init_node('vision_node', anonymous=True)

        # Setup device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.classifier_type = classifier
        self.detector_type = detector
        self.num_classes = classes

        self.script_dir = Path(__file__).parent

        # preprocessing transforms
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # Load detection model
        if self.detector_type == 'yolo':
            yolo_path = "../models/yolo/pm_28model.pt"
            model_path = self.script_dir / yolo_path
            print(model_path)
            self.yolo_model = YOLO(model_path)
            if self.yolo_model:
                rospy.loginfo('YOLO model loaded.')
            else:
                rospy.logerr('Error loading YOLO model.')
        elif self.detector_type == 'onnx':
            onnx_path = "../models/onnx/pm_28model.onnx"
            model_path = self.script_dir / onnx_path
            if not model_path.exists():
                rospy.logerr(f"ONNX model file not found: {model_path}")
                raise FileNotFoundError(f"ONNX model file not found: {model_path}")
            print(f"Loading ONNX model from: {model_path}")
            self.onnx_model = ONNXDetector(model_path)
            rospy.loginfo('Onnx model loaded.')
        else:
            rospy.logerr('Detector must be "yolo" or "onnx".')
    

        # Load classifiers based on number of classes and model type
        if classes == 4:
            if classifier == 'res':
                self.classifier_model, self.classifier_labels = self.load_res_model(num_classes=self.num_classes)
            elif classifier == 'conv':
                self.classifier_model, self.classifier_labels = self.load_conv4_model()
            elif classifier == 'eff':
                self.classifier_model, self.classifier_labels = self.load_eff_model(num_classes=self.num_classes)
            elif classifier == 'all':
                self.res_model, _ = self.load_res_model(num_classes=self.num_classes)
                self.conv_model, _ = self.load_conv4_model()
                self.eff_model, self.classifier_labels = self.load_eff_model(num_classes=self.num_classes)

        elif classes == 5:
            if classifier == 'res':
                self.classifier_model, self.classifier_labels = self.load_res_model(num_classes=self.num_classes)
            elif classifier == 'eff':
                self.classifier_model, self.classifier_labels = self.load_eff_model(num_classes=self.num_classes)



        # This subscribes to the image topic which is just a message with the path to the saved warp-D image
        self.sub = rospy.Subscriber('/image_topic', String, self.callback)

        self.pub = rospy.Publisher('/trajectory_topic', PointArray, queue_size=5)

        rospy.loginfo("Vision System Initialized.")


    def load_res_model(self, num_classes):
        try:
            if num_classes == 4:
                model_dir = "../models/resnet/resnet_4"
                model_path = self.script_dir / model_dir / 'res_best.pth'
                if not model_path.exists():
                    rospy.logerr(f'ResNet_4 model not found at {model_path}')
                    raise FileNotFoundError('ResNet model not found.')
                checkpoint = torch.load(model_path, map_location=torch.device(self.device))
                
                # Create model with simple fc layer for 4-class model
                model = torchvision.models.resnet18()
                num_ftrs = model.fc.in_features
                model.fc = nn.Linear(num_ftrs, num_classes)
                
            elif num_classes == 5:
                model_dir = "../models/resnet/resnet_5"
                model_path = self.script_dir / model_dir / "res_best_5.pth"
                if not model_path.exists():
                    rospy.logerr(f"ResNet model file not found: {model_path}")
                    raise FileNotFoundError(f"ResNet model file not found: {model_path}")
                checkpoint = torch.load(model_path, map_location=torch.device(self.device))
                
                # Create model with sequential fc layer for 5-class model
                model = torchvision.models.resnet18()
                num_ftrs = model.fc.in_features
                model.fc = nn.Sequential(
                    nn.Dropout(p=0.2),
                    nn.Linear(num_ftrs, num_classes)
                )

            class_mapping = checkpoint.get('class_mapping', {})
            # Convert keys to string to handle potential integer/string key differences
            # print(f"Classes: {class_names}")
            
            # Load model weights from checkpoint
            model.load_state_dict(checkpoint['model_state_dict'])
            model.to(self.device)
            model.eval()

            class_mappings = self.load_class_mappings(path=str(self.script_dir / model_dir))
            rospy.loginfo('Resnet model loaded.')
            return model, class_mappings
        except Exception as e:
            rospy.logerr(f"Failed to load ResNet model: {e}")
            return None, None
        


    def load_eff_model(self, num_classes):
        try:
            if num_classes == 4:
                model_dir = "../models/eff_net/eff_net4"
                model_path = self.script_dir / model_dir / "eff_best_4.pth"
                if not model_path.exists():
                    rospy.logerr(f"EfficientNet model file not found: {model_path}")
                    raise FileNotFoundError(f"EfficientNet model file not found: {model_path}")
                checkpoint = torch.load(model_path, map_location=torch.device(self.device))
                
                # Create model with same architecture
                model = torchvision.models.efficientnet_b0(weights=None)
                model.classifier = nn.Sequential(
                    nn.Dropout(p=0.3, inplace=True),
                    nn.Linear(in_features=1280, out_features=num_classes)
                )
            elif num_classes == 5:
                model_dir = "../models/eff_net/eff_best_5"
                model_path = self.script_dir / model_dir / "eff_best.pth"
                if not model_path.exists():
                    rospy.logerr(f"EfficientNet model file not found: {model_path}")
                    raise FileNotFoundError(f"EfficientNet model file not found: {model_path}")
                checkpoint = torch.load(model_path, map_location=torch.device(self.device))
                
                class_mapping = checkpoint.get('class_mapping', {})
                
                model = torchvision.models.efficientnet_b0(weights=torchvision.models.EfficientNet_B0_Weights.IMAGENET1K_V1)
                model.classifier = nn.Sequential(
                    nn.Dropout(p=0.3, inplace=True),
                    nn.Linear(in_features=1280, out_features=num_classes)
                )
                
                classifier_weights = {}
                for key, value in checkpoint['model_state_dict'].items():
                    if key.startswith('classifier'):
                        classifier_weights[key] = value
                
                model_dict = model.state_dict()
                model_dict.update(classifier_weights)
                model.load_state_dict(model_dict, strict=False)
                
                print("Loaded only classifier weights from checkpoint")

            model.to(self.device)
            model.eval()

            class_mappings = self.load_class_mappings(path=str(self.script_dir / model_dir))
            rospy.loginfo('EfficientNet model loaded.')
            return model, class_mappings
        
        
        except Exception as e:
            rospy.logerr(f"Failed to load EfficientNet model: {e}")
            return None, None
        
    def load_conv4_model(self):
        try:
            model_dir = "../models/convnext/best_conv_4"
            model_path = self.script_dir / model_dir / "conv_best_4.pth"
            if not model_path.exists():
                rospy.logerr(f"ConvNeXt model file not found: {model_path}")
                raise FileNotFoundError(f"ConvNeXt model file not found: {model_path}")
            checkpoint = torch.load(model_path, map_location=torch.device(self.device))
            
            class_mapping = checkpoint.get('class_mapping', {})
            class_names = [class_mapping[i] for i in range(len(class_mapping))]
            
            model = torchvision.models.convnext_small(weights=ConvNeXt_Small_Weights.IMAGENET1K_V1)
            num_ftrs = model.classifier[2].in_features
            model.classifier[2] = nn.Linear(num_ftrs, len(class_names))
            
            model.load_state_dict(checkpoint['model_state_dict'])
            model.to(self.device)
            model.eval()

            class_mappings = self.load_class_mappings(path=str(self.script_dir / model_dir))
            rospy.loginfo('ConvNeXt model loaded.')
            return model, class_mappings
        
        except Exception as e:
            rospy.logerr(f"Failed to load ConvNeXt model: {e}")
            return None, None

        
    def load_class_mappings(self, path):
        class_mapping_path = Path(path) / "class_mapping.json"
        try:
            if not class_mapping_path.exists():
                rospy.logerr(f"Class mapping file not found: {class_mapping_path}")
                raise FileNotFoundError(f"Class mapping file not found: {class_mapping_path}")
            with open(class_mapping_path, 'r') as f:
                class_mapping = json.load(f)
            class_labels = [class_mapping[str(i)] for i in range(len(class_mapping))]
            return class_labels
        except Exception as e:
            rospy.logerr(f"Failed to load class mappings: {e}")
            return None

    def preprocess_image(self, cv_image):
        # Convert BGR  to RGB
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Apply PyTorch transforms
        img = self.transform(img)
        img = img.unsqueeze(0)
        return img
    
    def classify_images(self, images: list, boxes: list):
        if self.classifier_model is None:
                    rospy.logerr('Classifier model not loaded.')
                    return
        if len(images) != len(boxes):
            rospy.logerr("Length of images and labels are not even.")
            return
        yolo_format_labels = []
        for idx, img in enumerate(images):
            try:
                processed_image = self.preprocess_image(img).to(self.device)

                with torch.no_grad():
                    outputs = self.classifier_model(processed_image)
                    _, predicted_class_idx = torch.max(outputs, 1)
                    # predicted_class = self.classifier_labels[predicted_class_idx.item()]
                    
                    yolo_format_labels.append([predicted_class_idx.item(), boxes[idx][0], boxes[idx][1], boxes[idx][2], boxes[idx][3]])

            except Exception as e:
                rospy.loginfo(f"Error classifying image.")

        return yolo_format_labels
    
    def all_classifier(self, images:list, boxes:list):
        if self.res_model is None or self.conv_model is None or self.eff_model is None:
            rospy.logerr("One of the classifier models is not loaded. ")
            return
        
        if len(images) != len(boxes):
            rospy.logerr("Length of images and labels are not even.")
            return
        
        yolo_format_labels = []
        for idx, img in enumerate(images):
            try:
                processed_image = self.preprocess_image(img).to(self.device)
                
                with torch.no_grad():
                    
                    res_outputs = self.res_model(processed_image)
                    conv_outputs = self.conv_model(processed_image)
                    eff_outputs = self.eff_model(processed_image)

                    _, res_predicted_class_idx = torch.max(res_outputs, 1)
                    _, conv_predicted_class_idx = torch.max(conv_outputs, 1)
                    _, eff_predicted_class_idx = torch.max(eff_outputs, 1)

                    if res_predicted_class_idx == conv_predicted_class_idx or res_predicted_class_idx == eff_predicted_class_idx:
                        predicted_class_idx = res_predicted_class_idx
                    elif conv_predicted_class_idx == res_predicted_class_idx or conv_predicted_class_idx == eff_predicted_class_idx:
                        predicted_class_idx = conv_predicted_class_idx

                    yolo_format_labels.append([predicted_class_idx.item(), boxes[idx][0], boxes[idx][1], boxes[idx][2], boxes[idx][3]])

            except Exception as e:
                rospy.loginfo(f"Error classifying image.")

        return yolo_format_labels

    def callback(self, msg):
       image_path = msg.data
       rospy.loginfo("Image received.")
       if self.detector_type == 'yolo':
           crops, boxes = my_crop_with_loaded_model(image_path=image_path, model=self.yolo_model, conf=0.1)
       elif self.detector_type == 'onnx':
           crops, boxes = my_crop_with_onnx(image_path=image_path, detector=self.onnx_model, conf=0.1, iou=0.2)

       if self.classifier_type != 'all':
           yolo_format_labels = self.classify_images(crops, boxes)
        
       else:
           yolo_format_labels = self.all_classifier(crops, boxes)
   
       objects = labels_to_world(yolo_format_labels)
       path = simulated_annealing(objects=objects, bins=bins_px, start_pos=start_position_px)
       classes_and_positions = return_path_with_class(path)
    #    print(classes_and_positions)

       trajectory = PointArray()
       trajectory.points = [ClassifiedPoint(class_id=cls, x=x, y=y) for (cls, x, y) in classes_and_positions] 

       self.pub.publish(trajectory)
       print('Path published.')

       
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        # For the vision system you can set classes equal to 4 or 5 depending if we want to include contaminants or not
        # If you do 4 classes you can set the classifier to 'res', 'eff', 'conv' or 'all'
        # which will make the classifier the respective classification model and 'all' will use all 3 and be redundant
        # If you use 5 classes you can only use 'res' or 'eff''
        # For the detector you can use 'yolo' or 'onnx'
        node = VisionSystemNode(classes=4, classifier='all', detector='onnx')
        node.run()

    except rospy.ROSInterruptException:
        pass