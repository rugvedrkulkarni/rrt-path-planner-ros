# RRT Path Planning for Turtlebot3 in ROS


This project implements a **Rapidly-Exploring Random Tree (RRT)** planner for the Turtlebot3 Burger robot in the Gazebo Stage 4 world.  
The planner generates a collision-free path on a grid map and uses a simple goal-following controller to drive the robot to the target.

---

##  Features
- RRT algorithm with obstacle checking
- Grid <-> Cartesian coordinate conversion
- Goal-following controller publishing `/cmd_vel`
- Obstacle augmentation tool (`ablate.py`) for robustness testing
- Path visualization with matplotlib
- Runs in **ROS Noetic** with Turtlebot3 Burger

---


## Results
**Gazebo Simulation**  
<img width="1920" height="1080" alt="gazebo_end_pos" src="https://github.com/user-attachments/assets/5f76797b-65d7-4045-b593-9e69e8544d23" />


**RRT Path Tree**  
<img width="653" height="547" alt="rrt_path" src="https://github.com/user-attachments/assets/9669eb0b-705c-4505-adef-b383cbc88ec6" />



##  How to Run
```bash
# Set model
export TURTLEBOT3_MODEL=burger

# Build and source
cd ~/catkin_ws
catkin build
source devel/setup.bash

# Launch
roslaunch rrt rrt.launch

---




