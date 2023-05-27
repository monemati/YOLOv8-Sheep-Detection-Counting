#!/usr/bin/env python3
import cv2
import os
from ultralytics import YOLO


def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)


# Load the YOLOv8 model
model = YOLO('resources/weights/yolov8m-sheep.pt')

# Open the video file
file_path = "resources/videos/aerial-sheep.mp4"
cap = cv2.VideoCapture(file_path)
unique_id=set()

while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model.track(frame, tracker="bytetrack.yaml") 
        img = results[0].plot()
        height, width, _ = img.shape
        # print(results)
        if  results[0].boxes.id !=  None:
            boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
            ids = results[0].boxes.id.cpu().numpy().astype(int)
            for box, id in zip(boxes, ids):
                # Check if the id is unique
                int_id =int(id)
                if  int_id  not  in  unique_id:
                    unique_id.add(int_id)               
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.line(img, (width - 500,25), (width,25), [85,45,255], 40)
            cv2.putText(img, f'Number of sheeps: {len(unique_id)}', (width - 500, 35), 0, 1, [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
            resized_img = ResizeWithAspectRatio(img, height=720)
            cv2.imshow('Detected Frame', resized_img)
            print(len(unique_id))
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
