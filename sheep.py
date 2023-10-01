#!/usr/bin/env python3
import cv2
import os
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Load the YOLOv8 model
model = YOLO('resources/weights/yolov8m-sheep.pt')
unique_id=set()

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

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/uav1/camera1/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image
    results = model.track(image, tracker="bytetrack.yaml", persist=True)
    img = results[0].plot()
    height, width, _ = img.shape
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
    cv2.waitKey(3)

def main():
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()

