import cv2
import mediapipe as mp
import os
import time as t
import math
import serial
import time

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

# --- state ------------------------------------------------------------------
cooldown_secs = 1.5
last_command_time = 0
# ---------------------------------------------------------------------------

# Calibration values for distance measurement
KNOWN_WIDTH = 16        # cm, average human face width
CALIB_DISTANCE_CM = 30.48  # 12 inches in cm
CALIB_PIXEL_WIDTH = 680    # Assumed pixel width at calibration distance
FOCAL_LENGTH = (CALIB_PIXEL_WIDTH * CALIB_DISTANCE_CM) / KNOWN_WIDTH 
Cam_CenterX = 960
Cam_CenterY = 540

print("Press 'q' to quit.")

with mp_face.FaceDetection(model_selection=1, min_detection_confidence=0.3) as face:
    while cap.isOpened():
        ok, frame = cap.read()
        frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=30)
        if not ok:
            print("Ignoring empty camera frame.")
            continue

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face.process(rgb)

        now = t.time()
        if now - last_command_time < cooldown_secs:
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

            # Process only the nearest face
            if nearest_face is not None:
                mp_drawing.draw_detection(frame, nearest_face)
                bbox = nearest_face.location_data.relative_bounding_box
                height, width, _ = frame.shape

                x_min = int(bbox.xmin * width)
                y_min = int(bbox.ymin * height)
                box_width = int(bbox.width * width)
                box_height = int(bbox.height * height)

                # Center of face
                x_center = x_min + box_width // 2
                y_center = y_min + box_height // 2

                # Estimate distance and angles
                distance_cm = (KNOWN_WIDTH * FOCAL_LENGTH) / box_width
                distance_in = distance_cm / 2.54
                lateral_dist_from_center = ((Cam_CenterX - x_center) / (box_width / 16))
                vertical_dist_from_center = ((Cam_CenterY - y_center) / (box_width / 16))
                dist_from_center = math.sqrt(lateral_dist_from_center**2 + vertical_dist_from_center**2)
                lateral_angle_to_face = math.atan(lateral_dist_from_center / distance_cm) * (180/3.14)
                vertical_angle_to_face = math.atan(vertical_dist_from_center / distance_cm) * (180/3.14)

                print(f"Nearest face center: ({x_center}, {y_center}) | "
                      f"Distance: {distance_cm:.2f} cm ({distance_in:.1f} in) | "
                      f"Abs. Dist {dist_from_center:.2f} cm | "
                      f"Lat. Angle {lateral_angle_to_face:.2f} deg | "
                      f"Vert. Angle {vertical_angle_to_face:.2f} deg")

                # Send angle to Arduino
                try:
                    arduino = serial.Serial('/dev/cu.usbserial-10', 100000)  # Change to your port
                    arduino.write(f"{lateral_angle_to_face:.2f}\n".encode())
                    time.sleep(.1)
                except Exception as e:
                    print(f"[Warning] Could not send to Arduino: {e}")

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
