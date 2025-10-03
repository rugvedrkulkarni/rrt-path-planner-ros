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


---

##  Results

**Gazebo Simulation (Turtlebot3 navigating in Stage 4 world)**  
![Gazebo Simulation](https://github.com/user-attachments/assets/4695a750-adb2-4e3b-add5-855305f7442e)

**RRT Tree and Planned Path**  
![RRT Path](https://github.com/user-attachments/assets/747cb666-4ea9-463b-b80e-a40dcf2c1c4f)


