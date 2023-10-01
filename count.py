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
unique_id=set()
# Open the image file
file_path = "resources/images/01.jpg"
results = model.track(file_path, tracker="bytetrack.yaml", persist=True)
img = results[0].plot()
height, width, _ = img.shape

boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
ids = results[0].boxes.id.cpu().numpy().astype(int)
for box, id in zip(boxes, ids):
    # Check if the id is unique
    int_id =int(id)
    if  int_id  not  in  unique_id:
        unique_id.add(int_id)
                    
cv2.line(img, (width - 500,25), (width,25), [85,45,255], 40)
cv2.putText(img, f'Number of sheeps: {len(unique_id)}', (width - 500, 35), 0, 1, [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
resized_img = ResizeWithAspectRatio(img, height=720)
cv2.imshow('Detected Frame', resized_img)
# Break the loop if 'q' is pressed
if cv2.waitKey(0) & 0xFF == ord("q"):
   exit()                   

