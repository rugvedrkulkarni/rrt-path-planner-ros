#!/usr/bin/env python3

import numpy as np
import yaml
import math
import random
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import os

# --- Coordinate Conversion ---
def cartesian_to_pixel(x, y, origin=(60, 60), resolution=0.05):
    px = int(x / resolution) + origin[0]
    py = origin[1] - int(y / resolution)
    return px, py

def pixel_to_cartesian(px, py, origin=(60, 60), resolution=0.05):
    x = (px - origin[0]) * resolution
    y = (origin[1] - py) * resolution
    return x, y

# -----------------------My understanding of the algorithm:--------------------------

# 1)robot has its starting point and nothing else
# 2)robot makes a plan to explore locations around him for n times 
# 3)he does it , while doing it he stores the safe locations in his mind
# 4)he finds another safe location(x_rand) close to the previous safe location(x_nearest)
# 5)he decides to go to that newly found safe location , but doesnt go there fully just a certain distance , calls it x_new
# 6)checks mentally if the journey from x nearest to x_new would be safe 
# 7) if yes it adds it to the tree and basically will go in that direction ,if not it will discard the whole idea and keep looking for new x_rand
# 8. This continues until it reaches close to the goal.
# 9. Finally, the robot follows the path from the tree using a simple goal-seeking behavior.

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

# --- Sampling Free Point ---
def sample_free(grid):
    while True:
        x = random.randint(0, grid.shape[1] - 1)
        y = random.randint(0, grid.shape[0] - 1)
        if grid[y, x] == 0:
            return x, y

# --- Nearest Node ---
def get_nearest_node(nodes, x_rand):
    nearest_node = nodes[0]
    min_dist = math.hypot(x_rand[0] - nearest_node.x, x_rand[1] - nearest_node.y)
    for node in nodes[1:]:
        dist = math.hypot(x_rand[0] - node.x, x_rand[1] - node.y)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node

# --- Steer ---
def steer(x_nearest, x_rand, step_size=10):
    dx = x_rand[0] - x_nearest.x
    dy = x_rand[1] - x_nearest.y
    distance = math.hypot(dx, dy)
    if distance < step_size:
        return x_rand
    theta = math.atan2(dy, dx)
    new_x = int(x_nearest.x + step_size * math.cos(theta))
    new_y = int(x_nearest.y + step_size * math.sin(theta))
    return new_x, new_y

# --- Obstacle Check ---
def is_obstacle_free(grid, x1, y1, x2, y2, clearance=4):
    dx = x2 - x1
    dy = y2 - y1
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return grid[y1, x1] == 0
    for i in range(steps + 1):
        t = i / steps
        x = int(x1 + t * dx)
        y = int(y1 + t * dy)
        for dx_clear in range(-clearance, clearance + 1):
            for dy_clear in range(-clearance, clearance + 1):
                nx = x + dx_clear
                ny = y + dy_clear
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                    if grid[ny, nx] != 0:
                        return False
                else:
                    return False
    return True


class GoalFollower:
    def __init__(self, waypoints):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_pose = None
        self.current_index = 0
        self.reached_goal_threshold = 0.1
        self.rate = rospy.Rate(10)
        self.waypoints = waypoints
        rospy.loginfo("Waypoints:")
        for i, (x, y) in enumerate(self.waypoints):
            rospy.loginfo(f"  {i}: ({x:.2f}, {y:.2f})")


    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        self.current_pose = (position.x, position.y, yaw)

    def run(self):
        while not rospy.is_shutdown() and self.current_index < len(self.waypoints):
            if self.current_pose is None:
                self.rate.sleep()
                continue

            goal_x, goal_y = self.waypoints[self.current_index]
            x, y, theta = self.current_pose
            rospy.loginfo(f"[{self.current_index}] Current Position: ({x:.2f}, {y:.2f}) Target: ({goal_x:.2f}, {goal_y:.2f})")

            dx = goal_x - x
            dy = goal_y - y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(angle_to_goal - theta)

            twist = Twist()
            if distance > self.reached_goal_threshold:
                twist.linear.x = max(0.1, 0.2 * distance)
                twist.angular.z = 0.8 * angle_diff
            else:
                self.current_index += 1
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


if __name__ == '__main__':
    rospy.init_node('rrt_with_goal_following')
    grid_file = rospy.get_param("grid")
    goal_cartesian = rospy.get_param("goal_position")
    package_path = os.path.dirname(os.path.realpath(__file__))
    grid_path = os.path.join(package_path, '../map', grid_file) # the grid with obstacles is saved as temp_map
    grid = np.load(grid_path)


    def get_robot_start_position():
        odom_msg = rospy.wait_for_message("/odom", Odometry)
        return odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y

    start_cartesian = get_robot_start_position()


    start_px = cartesian_to_pixel(*start_cartesian)
    goal_px = cartesian_to_pixel(*goal_cartesian)

    start_node = Node(*start_px)
    goal_node = Node(*goal_px)
    nodes = [start_node]

    n_iterations = 1000
    step_size = 10
    goal_threshold = 5
    goal_reached = False

    for _ in range(n_iterations):
        x_rand = sample_free(grid)
        x_nearest = get_nearest_node(nodes, x_rand)
        x_new = steer(x_nearest, x_rand, step_size)
        if is_obstacle_free(grid, x_nearest.x, x_nearest.y, x_new[0], x_new[1]):
            new_node = Node(x_new[0], x_new[1], x_nearest)
            nodes.append(new_node)
            if math.hypot(new_node.x - goal_px[0], new_node.y - goal_px[1]) < goal_threshold:
                goal_node = Node(goal_px[0], goal_px[1], new_node)
                nodes.append(goal_node)
                goal_reached = True
                break

    if goal_reached:
        path = []
        node = goal_node
        while node:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        waypoints = [pixel_to_cartesian(x, y) for x, y in path]
        filtered = [waypoints[0]]
        for p in waypoints[1:]:
            if math.hypot(p[0] - filtered[-1][0], p[1] - filtered[-1][1]) > 0.1:
                filtered.append(p)

                # ---- Visualization ----
        plt.imshow(grid, cmap='gray_r')
        for node in nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue', linewidth=0.5)

        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        plt.plot(path_x, path_y, 'r+', markersize=8, markeredgewidth=2.5, label='Path Waypoints')
        plt.scatter(*start_px, color='green', label='Start')
        plt.scatter(*goal_px, color='orange', label='Goal')
        plt.legend()
        plt.title("RRT Path")
        plt.show(block=False)
        plt.pause(5)  
        plt.close()


        follower = GoalFollower(filtered)
        follower.run()
    else:
        rospy.logerr("Failed to find a path to the goal.")



