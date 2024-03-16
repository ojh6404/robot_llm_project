# vlm_ros

ROS1 package for open-source Visual-Language-Model such as [LLaVA](https://github.com/haotian-liu/LLaVA.git) and [honeybee](https://github.com/kakaobrain/honeybee.git).

## Setup

### Prerequisite
This package is build upon
- ROS1 (Noetic)
- flask (communication between rosnode and docker inference server)
- docker and nvidia-container-toolkit (inference server)

### Build package
```bash
mkdir -p ~/ros/catkin_ws/src && cd ~/ros/catkin_ws/src
git clone https://github.com/ojh6404/vlm_ros.git
cd vlm_ros && docker build -t vlm_ros . # build docker inference server
cd ~/ros/catkin_ws && catkin b
```

## How to use
### 1. VQA
First, you need to launch docker inference server by
```bash
./run_docker -p 8888 -m honeybee
```
where
- `-p` or `--port` : which port to use.
- `-m` or `--model` : which model to use. Default is honeybee. `[llava, honeybee]`

and launch vqa node by
```bash
roslaunch vlm_ros sample_vqa.launch \
    input_image:=/kinect_head/rgb/image_rect_color \
    gui:=true
```

