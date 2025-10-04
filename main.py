import cv2
import torch
import numpy as np
from PIL import Image
import json
from transformers import (
    DistilBertForSequenceClassification,
    DistilBertTokenizerFast,
    pipeline
)
from ultralytics import YOLO
import threading
import time
from collections import deque

class ObjectRecognizer:
    def __init__(self, model_path="./intent_model", confidence_threshold=0.7):
        self.confidence_threshold = confidence_threshold
        
        # Load your trained text classification model
        print("🤖 Loading your trained model...")
        self.text_model = DistilBertForSequenceClassification.from_pretrained(model_path)
        self.tokenizer = DistilBertTokenizerFast.from_pretrained(model_path)
        self.text_classifier = pipeline(
            "text-classification",
            model=self.text_model,
            tokenizer=self.tokenizer,
            device=0 if torch.cuda.is_available() else -1
        )
        
        # Load YOLO for object detection
        print("🔍 Loading YOLO object detection model...")
        self.yolo_model = YOLO('yolo12n.pt')  # Using YOLOv8 nano (fastest)
        
        # Load label mappings
        try:
            with open(f"{model_path}/config.json", "r") as f:
                config = json.load(f)
                self.id2label = config.get("id2label", {})
                if isinstance(self.id2label, dict):
                    # Convert string keys to integers
                    self.id2label = {int(k): v for k, v in self.id2label.items()}
                print(f"✅ Loaded labels: {self.id2label}")
        except:
            print("⚠️ Could not load label mappings, using default")
            self.id2label = {0: "PEN", 1: "PENCIL"}
        
        # Object tracking
        self.detection_history = deque(maxlen=10)
        self.current_object = None
        self.confidence_score = 0.0
        
    def detect_objects_yolo(self, frame):
        """Detect objects in frame using YOLO"""
        results = self.yolo_model(frame, verbose=False)
        detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    confidence = box.conf.item()
                    if confidence > self.confidence_threshold:
                        class_id = int(box.cls.item())
                        class_name = self.yolo_model.names[class_id]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        detections.append({
                            'class_name': class_name,
                            'confidence': confidence,
                            'bbox': [x1, y1, x2, y2],
                            'class_id': class_id
                        })
        
        return detections
    
    def analyze_object_with_text(self, detections):
        """Use your trained model to classify objects based on their names"""
        if not detections:
            return None, 0.0
        
        # Get the most confident detection
        best_detection = max(detections, key=lambda x: x['confidence'])
        object_name = best_detection['class_name']
        
        # Use your trained model to classify the object
        try:
            result = self.text_classifier(object_name)[0]
            label = result['label']
            confidence = result['score']
            
            # Map to your custom labels if needed
            if label in self.id2label.values():
                return label, confidence
            else:
                # If the label doesn't match, return the YOLO class name
                return object_name, confidence
                
        except Exception as e:
            print(f"⚠️ Error in text classification: {e}")
            return object_name, best_detection['confidence']
    
    def update_detection_history(self, current_object, confidence):
        """Update detection history for stability"""
        self.detection_history.append((current_object, confidence))
        
        # Get the most frequent detection in history
        if len(self.detection_history) >= 5:
            objects = [obj for obj, conf in self.detection_history]
            most_common = max(set(objects), key=objects.count)
            frequency = objects.count(most_common) / len(objects)
            
            if frequency > 0.6:  # 60% consensus
                avg_confidence = np.mean([conf for obj, conf in self.detection_history if obj == most_common])
                return most_common, avg_confidence
        
        return current_object, confidence
    
    def process_frame(self, frame):
        """Process a single frame and return annotated frame"""
        # Detect objects with YOLO
        detections = self.detect_objects_yolo(frame)
        
        if detections:
            # Analyze with your trained model
            current_object, confidence = self.analyze_object_with_text(detections)
            
            # Update history for stable detection
            self.current_object, self.confidence_score = self.update_detection_history(
                current_object, confidence
            )
            
            # Draw bounding boxes and labels
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                class_name = detection['class_name']
                conf = detection['confidence']
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label background
                label = f"{class_name}: {conf:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                            (x1 + label_size[0], y1), (0, 255, 0), -1)
                
                # Draw label text
                cv2.putText(frame, label, (x1, y1 - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return frame

def main():
    # Initialize the recognizer
    recognizer = ObjectRecognizer()
    
    # Initialize webcam
    print("📷 Initializing webcam...")
    cap = cv2.VideoCapture(0)
    
    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap.isOpened():
        print("❌ Error: Could not open webcam")
        return
    
    print("🎥 Webcam started. Press 'q' to quit, 's' to save image")
    
    # FPS calculation
    fps_counter = 0
    fps_time = time.time()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ Error: Could not read frame")
                break
            
            # Process frame
            processed_frame = recognizer.process_frame(frame)
            
            # Display current object info
            if recognizer.current_object:
                status_text = f"Object: {recognizer.current_object} ({recognizer.confidence_score:.2f})"
                cv2.putText(processed_frame, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(processed_frame, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
            
            # Calculate and display FPS
            fps_counter += 1
            if time.time() - fps_time >= 1.0:
                fps = fps_counter
                fps_counter = 0
                fps_time = time.time()
                
            cv2.putText(processed_frame, f"FPS: {fps_counter}", (10, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(processed_frame, f"FPS: {fps_counter}", (10, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
            
            # Display instructions
            cv2.putText(processed_frame, "Press 'q' to quit, 's' to save", (10, 450), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(processed_frame, "Press 'q' to quit, 's' to save", (10, 450), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Show the frame
            cv2.imshow('Object Recognition', processed_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save current frame
                timestamp = int(time.time())
                filename = f"capture_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"💾 Saved image as {filename}")
    
    except KeyboardInterrupt:
        print("🛑 Interrupted by user")
    
    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        print("✅ Application closed")

if __name__ == "__main__":
    main()