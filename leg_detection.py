import cv2
import mediapipe as mp
import os

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

cap = cv2.VideoCapture(2)

with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:

    while cap.isOpened():
        success, image = cap.read()
        image_height, image_width, _ = image.shape

        if not success:
            print("Ignoring empty camera frame.")
            continue

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            
            x_cordinate = list()
            y_cordinate = list()

            for id, lm in enumerate(results.pose_landmarks.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                x_cordinate.append(cx)
                y_cordinate.append(cy)
                
            cv2.rectangle(img= image,
                        pt1= (min(x_cordinate), max(y_cordinate)),
                        pt2 = (max(x_cordinate), min(y_cordinate)),
                        color= (0,255,0),
                        thickness= 2)

            right_leg = [int(results.pose_landmarks.landmark[27].x * image_width), int(results.pose_landmarks.landmark[27].y * image_height)]
            left_leg = [int(results.pose_landmarks.landmark[28].x * image_width), int(results.pose_landmarks.landmark[28].y * image_height)]

            right_leg_bounding = [int(results.pose_landmarks.landmark[31].x * image_width), int(results.pose_landmarks.landmark[23].y * image_height),
                                  int(results.pose_landmarks.landmark[29].x * image_width), int(results.pose_landmarks.landmark[31].y * image_height)]
            cv2.rectangle(img= image,
                        pt1= (right_leg_bounding[0], right_leg_bounding[1]),
                        pt2 = (right_leg_bounding[2], right_leg_bounding[3]),
                        color= (0,0,255),
                        thickness= 2)
            
            left_leg_bounding = [int(results.pose_landmarks.landmark[32].x * image_width), int(results.pose_landmarks.landmark[24].y * image_height),
                                 int(results.pose_landmarks.landmark[30].x * image_width), int(results.pose_landmarks.landmark[32].y * image_height)]
            
            cv2.rectangle(img= image,
                        pt1= (left_leg_bounding[0], left_leg_bounding[1]),
                        pt2 = (left_leg_bounding[2], left_leg_bounding[3]),
                        color= (0,0,255),
                        thickness= 2)
            
            print(f"Right Leg: {right_leg}")
            print(f"Right Leg Bounding: {right_leg_bounding}")
            print(f"Left Leg: {left_leg}")
            print(f"Left Leg Bounding: {left_leg_bounding}")

            x_cordinate.clear()
            y_cordinate.clear()

        cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))

        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()