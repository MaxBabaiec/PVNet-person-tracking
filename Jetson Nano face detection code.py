import cv2
import mediapipe as mp
import os
import time as t
import math
import serial

# Suppress TensorFlow logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# MediaPipe GPU setup
mp_face = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# Camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[Error] Camera could not be opened.")
    exit()

# Set camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# State
last_command_time     = 0
command_cooldown_secs = 1.5

# Calibration
KNOWN_WIDTH = 16        # cm, average human face width
CALIB_DISTANCE_CM = 30.48  # 12 inches in cm
CALIB_PIXEL_WIDTH = 680
FOCAL_LENGTH = (CALIB_PIXEL_WIDTH * CALIB_DISTANCE_CM) / KNOWN_WIDTH 
Cam_CenterX = 960
Cam_CenterY = 540

print("Press 'q' to quit.")

with mp_face.FaceDetection(model_selection=1, min_detection_confidence=0.4) as face:
    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            continue

        # Flip and convert to RGB
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = face.process(rgb)
        now = t.time()
        if now - last_command_time < command_cooldown_secs:
            continue

        nearest_face = None
        min_distance = float('inf')

        if results.detections:
            # Find nearest face
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                height, width, _ = frame.shape
                box_width = int(bbox.width * width)
                if box_width > 0:
                    distance_cm = (KNOWN_WIDTH * FOCAL_LENGTH) / box_width
                    if distance_cm < min_distance:
                        min_distance = distance_cm
                        nearest_face = detection

            if nearest_face is not None:
                mp_drawing.draw_detection(frame, nearest_face)
                bbox = nearest_face.location_data.relative_bounding_box
                height, width, _ = frame.shape

                x_min = int(bbox.xmin * width)
                y_min = int(bbox.ymin * height)
                box_width = int(bbox.width * width)
                box_height = int(bbox.height * height)

                # Center
                x_center = x_min + box_width // 2
                y_center = y_min + box_height // 2

                # Distance & angles
                distance_cm = (KNOWN_WIDTH * FOCAL_LENGTH) / box_width
                lateral_dist = (Cam_CenterX - x_center) / (box_width / 16)
                vertical_dist = (Cam_CenterY - y_center) / (box_width / 16)
                dist_from_center = math.sqrt(lateral_dist**2 + vertical_dist**2)
                lateral_angle = math.atan(lateral_dist / distance_cm) * (180 / 3.1416)
                vertical_angle = math.atan(vertical_dist / distance_cm) * (180 / 3.1416)

                print(f"Nearest face center: ({x_center}, {y_center}) | "
                      f"Distance: {distance_cm:.2f} cm | "
                      f"Lateral Angle: {lateral_angle:.2f} deg | "
                      f"Vertical Angle: {vertical_angle:.2f} deg")

                # Send lateral angle to Arduino
                try:
                    arduino = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0.1)
                    arduino.write(f"{lateral_angle:.2f}\n".encode())
                    arduino.close()
                except serial.SerialException:
                    print("[Warning] Arduino not connected or busy.")

                # Draw
                cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"{int(distance_cm)} cm", (x_center, y_center - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"{int(lateral_angle)} deg", (x_center, y_center + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Facial Recognition', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
