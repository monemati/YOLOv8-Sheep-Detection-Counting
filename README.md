# YOLOv8 Aerial Sheep Detection and Counting
YOLOv8 Aerial Sheep Detection and Counting. Simulated on Gazebo.

## Introduction
This repository contains a Sheep Detector and Counter trained by [YOLOv8](https://github.com/ultralytics/ultralytics) algorithm with [Sheep Dataset](https://universe.roboflow.com/riis/aerial-sheep/dataset/1) from Roboflow.

## Train Results
Train results on YOLOv8n. Weights are provided in resources/weights direcotry.

![alt text](/resources/demo/val_batch1_pred.jpg "YOLOv8n Train Result")

![alt text](/resources/demo/results.png "YOLOv8n Train Result")

## Installation
### Create a virtual environment
```commandline
# create
python -m venv yolov8-sheep

# activate
source yolov8-sheep/bin/activate
```

### Clone repository
```commandline
git clone https://github.com/monemati/YOLOv8-Sheep-Detection-Counting.git
cd YOLOv8-Sheep-Detection-Counting
```

### Install packages
```commandline
pip install -e '.[dev]'
```

## Run
```commandline
# On image
python count.py

# On Video
python track.py
```
- Change file_path to your desired files. Sample files are provided in resources/images and resources/videos direcotries.

## Simulate in Gazebo
- You need to follow [this tutorial](https://github.com/monemati/multiuav-gazebo-simulation) to setup test environment.
- You can use Models and Worlds provided in resources/models and resources/worlds direcotries.
- Open a terminal and use the command below to launch your world (this will launch gazebo):
```
roslaunch gazebo_ros agriculture.launch
```
- Open a new terminal and run a UAV:
```
cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I0
```
- After seeing "APM: EKF2 IMU0 is using GPS" message in console, you can use the commands below to takeoff:
```
mode guided
arm throttle
takeoff 40
```
- Now in a new terminal use the command below:
```
python sheep.py
```
- You can watch the demo from [this file](https://github.com/monemati/YOLOv8-Sheep-Detection-Counting/blob/main/resources/demo/Gazebo-Sheep-Detector-Counting-Demo.mp4)

## Results

![alt text](/resources/demo/Gazebo-Sheep-Detector-Counting.png "Gazebo Sheep Detector Counting")

![alt text](/resources/demo/Aerial-Sheep-01.png "Aerial Sheep")
  
## Acknowledgement
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/
- https://github.com/ArduPilot/ardupilot
