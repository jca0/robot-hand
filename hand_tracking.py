import cv2
import mediapipe as mp
import numpy as np
import serial
import time

arduino = serial.Serial('/dev/cu.usbmodem1101', 9600) # change according to arduino port
time.sleep(2)

# 0 = continuity cam, 1 = webcam
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()


def ratio_to_angle(ratio, min_ratio=0.5, max_ratio=1.5):
    ratio = np.clip(ratio, min_ratio, max_ratio)
    norm = (ratio - min_ratio) / (max_ratio - min_ratio)
    angle = int((1 - norm) * 180)
    return angle

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

while True:
    success, frame = cap.read()
    if not success:
        break
    frame = cv2.flip(frame, 1)
    RGB_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(RGB_frame)
    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            lm = hand_landmarks.landmark

            # thumb landmarks
            thumb_tip = lm[4]
            thumb_ip = lm[3]
            thumb_mcp = lm[2]
            thumb_cmc = lm[1]
            wrist = lm[0]

            # compute thumb bend
            # Use vectors: thumb_ip -> thumb_tip and thumb_ip -> thumb_mcp
            v1 = np.array([thumb_tip.x - thumb_ip.x, thumb_tip.y - thumb_ip.y])
            v2 = np.array([thumb_mcp.x - thumb_ip.x, thumb_mcp.y - thumb_ip.y])
            dot = np.dot(v1, v2)
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            if norm1 * norm2 != 0:
                angle_rad = np.arccos(dot / (norm1 * norm2))
            else:
                angle_rad = 0
            thumb_bend_deg = np.degrees(angle_rad)
            # Map thumb bend (assume ~180° when extended, ~90° when bent)
            thumb_bend_servo = int(np.interp(thumb_bend_deg, [100, 160], [180, 0]))
            
            # Compute thumb rotation using vector from thumb_mcp to thumb_tip
            thumb_vector = np.array([thumb_tip.x - thumb_mcp.x, thumb_tip.y - thumb_mcp.y])
            thumb_rot = np.arctan2(thumb_vector[1], thumb_vector[0])
            thumb_rot_deg = int(np.interp(thumb_rot, [-np.pi, np.pi], [0, 120]))

            # control other fingers using ratio
            fingers = [(8, 6, 5), (12, 10, 9), (16, 14, 13), (20, 18, 17)]
            angles = []
            for tip_idx, pip_idx, mcp_idx in fingers:
                tip = lm[tip_idx]
                pip = lm[pip_idx]
                mcp = lm[mcp_idx]
                tip_to_mcp = np.linalg.norm(np.array([tip.x, tip.y]) - np.array([mcp.x, mcp.y]))
                pip_to_mcp = np.linalg.norm(np.array([pip.x, pip.y]) - np.array([mcp.x, mcp.y]))
                ratio = tip_to_mcp / pip_to_mcp if pip_to_mcp != 0 else 0
                angle = ratio_to_angle(ratio)
                angles.append(angle)

            # Order: thumb rotation, index, middle, ring, pinky, thumb bend
            servo_angles = [thumb_rot_deg] + angles + [thumb_bend_servo]
            angle_str = ",".join(str(a) for a in servo_angles)
            arduino.write(f"{angle_str}\n".encode())

            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            for i, angle in enumerate(servo_angles):
                cv2.putText(frame, f"S{i}: {angle}", (10, 30 + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Webcam", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
