#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from robot_msgs.msg import camera
from ultralytics import YOLO

class TrashDetector:
    def __init__(self):
        rospy.init_node('trash_detector', log_level=rospy.INFO)

        rospy.loginfo("Loading YOLO model...")
        self.model = YOLO('/home/roobics/titanium-bot/titanium_ws/src/camera_vision/scripts/trash_detect_v1.pt')
        
        self.camera_center_x = 320
        self.camera_center_y = 190
        
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.5)
        
        self.display = rospy.get_param("~display", True)
        
        self.camera_pub = rospy.Publisher('/sensor/camera', camera, queue_size=10)
        self.camera_msg = camera()
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        
        if not self.cap.isOpened():
            rospy.logerr("Unable to open camera!")
            rospy.signal_shutdown("Camera initialization failed")
            return
            
        rospy.loginfo("Trash detector node initialized")
        rospy.loginfo(f"Display: {self.display}, Confidence threshold: {self.confidence_threshold}")

    def colourBoundingBox(self, class_id):
        if class_id == "daun":
            r, g, b = 199, 252, 0
        elif class_id == "ferro":
            r, g, b = 254, 0, 86
        elif class_id == "kertas":
            r, g, b = 0, 255, 206
        elif class_id == "nonferro":
            r, g, b = 255, 128, 0
        elif class_id == "plastik":
            r, g, b = 134, 34, 255
        else:
            r, g, b = 255, 255, 255
        return r, g, b

    def drawReticle(self, frame, cx, cy):
        x_length = 10
        cv2.line(frame, (cx - x_length//2, cy - x_length//2), 
                (cx + x_length//2, cy + x_length//2), 
                (0, 255, 255), 2)
        cv2.line(frame, (cx - x_length//2, cy + x_length//2), 
                (cx + x_length//2, cy - x_length//2), 
                (0, 255, 255), 2)

    def infer(self):
        ret, frame = self.cap.read()
        
        if not ret:
            rospy.logwarn("Unable to read from camera!")
            return None, False, 0, 0, [0, 0, 0, 0], [0, 0]

        frame = cv2.flip(frame, 1)

        results = self.model.predict(frame, conf=self.confidence_threshold, verbose=False)

        detected_trash = False
        trash_type = 0
        closest_trash_x = 0
        closest_trash_y = 0
        closest_bbox = [0, 0, 0, 0]
        closest_centroid = [0, 0]
        min_distance = float('inf')
        display_frame = frame.copy() if self.display else None

        for result in results:
            if result.boxes is not None and len(result.boxes) > 0:
                detected_trash = True
                
                for box in result.boxes:
                    class_id = result.names[box.cls[0].item()]
                    cords = box.xyxy[0].tolist()
                    cords = [round(x) for x in cords]
                    conf = round(box.conf[0].item(), 2)

                    cx = int((cords[0] + cords[2]) / 2.0)
                    cy = int((cords[1] + cords[3]) / 2.0)

                    distance = np.sqrt((cx - self.camera_center_x)**2 + (cy - self.camera_center_y)**2)

                    if distance < min_distance:
                        min_distance = distance
                        closest_trash_x = cx
                        closest_trash_y = cy
                        closest_bbox = [cords[0], cords[1], cords[2], cords[3]]
                        closest_centroid = [cx, cy]
                    
                    if class_id == "daun":
                        trash_type = 1
                    elif class_id == "ferro":
                        trash_type = 2
                    elif class_id == "kertas":
                        trash_type = 3
                    elif class_id == "nonferro":
                        trash_type = 4
                    elif class_id == "plastik":
                        trash_type = 5
                    else:
                        trash_type = 0

                    if self.display:
                        r, g, b = self.colourBoundingBox(class_id)
                        
                        cv2.rectangle(display_frame, 
                                    (cords[0], cords[1]), 
                                    (cords[2], cords[3]), 
                                    (b, g, r), 
                                    2)
                        
                        label = f"{class_id}, {conf}, ({cx},{cy})"
                        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        
                        cv2.rectangle(display_frame, 
                                    (cords[0], cords[1] - text_size[1] - 5),
                                    (cords[0] + text_size[0], cords[1] - 5),
                                    (0, 0, 0), cv2.FILLED)
                        
                        cv2.putText(display_frame, 
                                    label, 
                                    (cords[0], cords[1] - 7), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.5, 
                                    (b, g, r), 
                                    1)
                        
                        # Draw centroid point
                        cv2.circle(display_frame, (cx, cy), 3, (0, 0, 255), -1)

        if self.display and display_frame is not None:
            self.drawReticle(display_frame, self.camera_center_x, self.camera_center_y)
        
            if detected_trash:
                cv2.line(display_frame, 
                        (self.camera_center_x, self.camera_center_y),
                        (closest_trash_x, closest_trash_y),
                        (255, 0, 255), 2)
                
                cv2.circle(display_frame, (closest_trash_x, closest_trash_y), 5, (255, 255, 0), -1)
                
                x_distance = closest_trash_x - self.camera_center_x
                y_distance = self.camera_center_y - closest_trash_y

                center_text = f"Camera Center: ({self.camera_center_x},{self.camera_center_y})"
                cv2.putText(display_frame, center_text, (20, display_frame.shape[0] - 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                euclidean_text = f"Euclidean: {min_distance:.1f} px"
                x_text = f"X-distance: {x_distance:.1f} px"
                y_text = f"Y-distance: {y_distance:.1f} px"
                
                cv2.putText(display_frame, euclidean_text, (20, display_frame.shape[0] - 80),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(display_frame, x_text, (20, display_frame.shape[0] - 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(display_frame, y_text, (20, display_frame.shape[0] - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                x_dir = "RIGHT" if x_distance > 0 else "LEFT"
                y_dir = "UP" if y_distance > 0 else "DOWN"
                
                direction_text = f"Direction: {x_dir}, {y_dir}"
                cv2.putText(display_frame, direction_text, (20, display_frame.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                cv2.putText(display_frame, "No trash detected!", (20, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return display_frame, detected_trash, trash_type, closest_trash_x, closest_trash_y, closest_bbox, closest_centroid

    def run(self):
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            display_frame, detected, trash_type, cx, cy, bbox, centroid = self.infer()
            
            self.camera_msg.trashDetected = detected
            self.camera_msg.trashType = trash_type
            self.camera_msg.cameraCenterX = self.camera_center_x
            self.camera_msg.cameraCenterY = self.camera_center_y
            
            if detected:
                self.camera_msg.closestTrashX = int(self.camera_center_x - cx)
                self.camera_msg.closestTrashY = int(cy - self.camera_center_y)
                self.camera_msg.bbox = bbox
                self.camera_msg.centroid = centroid
                rospy.logdebug(f"Detected trash at ({cx}, {cy})")
            else:
                self.camera_msg.closestTrashX = 0
                self.camera_msg.closestTrashY = 0
                self.camera_msg.bbox = [0, 0, 0, 0]
                self.camera_msg.centroid = [0, 0]
            
            self.camera_pub.publish(self.camera_msg)
            
            if self.display and display_frame is not None:
                cv2.imshow('YOLO Object Detection', display_frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC key
                    rospy.signal_shutdown("ESC pressed")
                    break
            
            rate.sleep()

    def cleanup(self):
        self.cap.release()
        if self.display:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = TrashDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    finally:
        detector.cleanup()
