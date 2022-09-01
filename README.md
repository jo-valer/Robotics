# Robotics project - _A mobile robot to pick up LEGO bricks_.
This is the _Fundamentals of Robotics'_ project by Pietro Fronza, Stefano Genetti, and [**Giovanni Valer**](https://github.com/jo-valer).

ℹ We have written the code mainly in **C++** (for the part regarding kinematics and trajectory planning) and **Python** (for object classification and localization). For any info about the implementation solutions, read the paper: <a href="https://github.com/jo-valer/Robotics/blob/main/Robotics_Project.pdf">Robotics_Project.pdf</a>.

## 🎥 Videos
It might be useful to have a look at some simulations we have done. You can find them on YouTube at the following links:
- <a href="https://youtu.be/23n-PxkJd8o">▶️ **Assignment 1**</a>
- <a href="https://youtu.be/45ijPx6vJCo">▶️ **Assignment 2**</a>
- <a href="https://youtu.be/dLTMpczgzr8">▶️ **Assignment 3**</a>

## 📁 Repository content 
Here you can find three different catkin workspaces, each one for a different assignment. All goals have been achieved.

⚠️ If you want to download the code and run it, make sure you comply with the <a href="#requirements">requirements</a>. Furthermore, you need to give execution permissions to 3 python scripts, namely: `src/my_world/world/lego_spawner.py`, `src/robotic_vision/src/localize_listener.py`, and `src/robotic_vision/src/yolov5/my_detect.py`. If you are not on a native linux machine you may need to directly create those files (then copying into them the content from this repo).


## <a href="https://github.com/jo-valer/Robotics/tree/main/Assignment_1">`Assignment_1`</a>
The whole environment has to be launched with:
  ```sh
  roslaunch my_world startcomplete.launch
  ```
After having un-paused the simulation, you can run the controller:
  ```sh
  rosrun mir_controller mir_controller
  ```
You will find the recognized bricks directly printed on the shell, and also in `OUTPUT.txt`. Here is an example with 4 bricks:
  ```java
  LEGO   class: 7,   name: X1_Y4_Z1,         x: 0.121254,   y: -2.0017
  LEGO   class: 5,   name: X1_Y3_Z2,         x: -0.120827,  y: -2.00102
  LEGO   class: 6,   name: X1_Y3_Z2_FILLET,  x: 1.86269,    y: 1.9847
  LEGO   class: 1,   name: X1_Y2_Z1,         x: 2.12052,    y: 1.9992
  ```


## <a href="https://github.com/jo-valer/Robotics/tree/main/Assignment_2">`Assignment_2`</a>
Basically, everything works as in the previous assignment, except for the robot, that now carries the bricks to their basket. The commands and the output are the same.


## <a href="https://github.com/jo-valer/Robotics/tree/main/Assignment_3">`Assignment_3`</a>
For the last assignment we decided to have a much more complicated environment, so now the world is stored as a map in a file: <a href="https://github.com/jo-valer/Robotics/tree/main/Assignment_3/src/my_world/src/map.txt">`src/my_world/src/map.txt`</a>.

To run this simulation, everything works as previously seen, but now the output has to be slightly different, since there can be up to 3 different bricks in the same target area:
  ```java
  AREA: 3 - LEGO  class: 7,   name: X1_Y4_Z1,         x: 0.121254,   y: -2.0017
  AREA: 3 - LEGO  class: 5,   name: X1_Y3_Z2,         x: -0.120827,  y: -2.00102
  
  AREA: 1 - LEGO  class: 1,   name: X1_Y2_Z1,         x: 2.12052,    y: 1.9992
  AREA: 1 - LEGO  class: 6,   name: X1_Y3_Z2_FILLET,  x: 1.86269,    y: 1.9847
  
  AREA: 2 - LEGO  class: 8,   name: X1_Y4_Z2,         x: 3.10256,    y: 4.01493
  AREA: 2 - LEGO  class: 8,   name: X1_Y4_Z2,         x: 2.91906,    y: 4.11383
  AREA: 2 - LEGO  class: 8,   name: X1_Y4_Z2,         x: 2.92037,    y: 3.90152
  
  AREA: 4 - LEGO  class: 6,   name: X1_Y3_Z2_FILLET,  x: -2.07731,   y: 1.06888
  AREA: 4 - LEGO  class: 0,   name: X1_Y1_Z2,         x: -2.0487,    y: 0.887195
  AREA: 4 - LEGO  class: 1,   name: X1_Y2_Z1,         x: -1.87581,   y: 1.04202
  ```
So that we can know how the robot explored the environment. In this example, it firstly visited area 3, then area 1 and 2, and at the end area 4.


## Requirements
- **Python 3**
- **ROS Noetic**
- **Ubuntu 20.04**
- After having cloned the repository:
```sh
cd src/robotic_vision/src/yolov5/
```
```sh
pip install -qr requirements.txt
```
