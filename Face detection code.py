import cv2
import mediapipe as mp
import os
import time as t
import joblib
import numpy as np
from pathlib import Path
import screen_brightness_control as sbc
import math
import serial

# Suppress TensorFlow logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# MediaPipe setup
mp_face = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# Camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[Error] Camera could not be opened.")
    exit()

# Load ASL model
model_path = Path("model/asl_knn_model.joblib")
encoder_path = Path("model/label_encoder.joblib")

if model_path.exists() and encoder_path.exists():
    asl_model = joblib.load(model_path)
    label_encoder = joblib.load(encoder_path)
    print("[Model] ASL recognition model loaded.")
else:
    asl_model = None
    label_encoder = None
    print("[Warning] No ASL model found. ASL recognition will be disabled.")

# --- state ------------------------------------------------------------------
active_read           = False
cooldown_secs         = 3.0
last_event_time       = 0
last_printed          = None
command_cooldown_secs = 1.5
last_command_time     = 0
asl_active            = False
last_asl_letter       = None
asl_start_time        = 0
# ---------------------------------------------------------------------------

# Calibration values for distance measurement
KNOWN_WIDTH = 16        # cm, average human face width
CALIB_DISTANCE_CM = 30.48  # 12 inches in cm
CALIB_PIXEL_WIDTH = 680    # Assumed pixel width at calibration distance
FOCAL_LENGTH = (CALIB_PIXEL_WIDTH * CALIB_DISTANCE_CM) / KNOWN_WIDTH 
Cam_CenterX = 960
Cam_CenterY = 540

print("Press 'q' to quit.")

with mp_face.FaceDetection(model_selection=1, min_detection_confidence=0.4) as face:
    while cap.isOpened():
        ok, frame = cap.read()
        frame = cv2.convertScaleAbs(frame, alpha=1.3, beta=70)

        if not ok:
            print("Ignoring empty camera frame.")
            continue

        frame = cv2.flip(frame, 1)
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face.process(rgb)

        now = t.time()
        if now - last_command_time < command_cooldown_secs:
            continue

        if results.detections:
            for detection in results.detections:
                mp_drawing.draw_detection(frame, detection)


                bbox = detection.location_data.relative_bounding_box
                height, width, _ = frame.shape

                x_min = int(bbox.xmin * width)
                y_min = int(bbox.ymin * height)
                box_width = int(bbox.width * width)
                box_height = int(bbox.height * height)

                # Center of face
                x_center = x_min + box_width // 2
                y_center = y_min + box_height // 2

                # Estimate distance
                if box_width > 0:
                    distance_cm = (KNOWN_WIDTH * FOCAL_LENGTH) / box_width
                    distance_in = distance_cm / 2.54
                    lateral_dist_from_center = ((Cam_CenterX - x_center) / (box_width / 16)) # cm
                    vertical_dist_from_center = ((Cam_CenterY - y_center) / (box_width / 16)) # cm
                    dist_from_center = math.fabs(math.sqrt(math.pow(lateral_dist_from_center, 2) + math.sqrt(math.pow(vertical_dist_from_center, 2))))
                    lateral_angle_to_face = math.atan(lateral_dist_from_center / distance_cm) * (180/3.14)
                    vertical_angle_to_face = math.atan(vertical_dist_from_center / distance_cm) * (180/3.14)
                    print(f"Face center: ({x_center}, {y_center}) | Distance: {distance_cm:.2f} cm ({distance_in:.1f} in) | Abs. Dist {dist_from_center:.2f} cm | Lat. Angle {lateral_angle_to_face:.2f} deg | Vert. Angle {vertical_angle_to_face:.2f} deg")
                    arduino = serial.Serial('/dev/cu.usbserial-10', 1000000)  # Change to your port
                    arduino.write(f"{lateral_angle_to_face:.2f}\n".encode())  # Python side
                else:
                    print("Face width is 0, can't estimate distance.")

                # Draw center and distance
                cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"{int(distance_cm)} cm", (x_center, y_center - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"{int(lateral_angle_to_face)} deg", (x_center, y_center + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                
                
                



        cv2.imshow('Facial Recognition', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
