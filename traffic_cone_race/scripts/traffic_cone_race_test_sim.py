import turtle
import math
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import LineString, Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from race.msg import drive_values

# PID controller parameters
Kp = 1.5
Ki = 0.1
Kd = 0.01

# Time step for derivative term calculation
dt = 0.1

# Set up the turtle
robot = turtle.Turtle()
robot.shape("turtle")
robot.speed(0)

prev_error = 0.0
integral = 0.0
min_dis = 0
close_goal_point = []

def cal_steering():
    global min_dis, close_goal_point, prev_error, integral

    wheel_base = 1.04

    if len(close_goal_point) > 1:
        # Calculate minimum distance to closest goal point
        error = min_dis

        # Calculate proportional, integral, and derivative terms
        p_term = Kp * error
        integral += error * dt
        i_term = Ki * integral
        derivative = (error - prev_error) / dt
        d_term = Kd * derivative

        # Calculate the PID control signal
        pid_output = p_term + i_term + d_term

        # Update the previous error for the next iteration
        prev_error = error

        # Calculate the steering angle using the PID control signal
        steering_cal = -math.atan((2 * close_goal_point[0][1] * wheel_base) / (min_dis * min_dis)) * (180 / math.pi)

        # Add the PID control signal to the calculated steering angle
        steering_cal += pid_output

        return steering_cal
    else:
        return 0.0

# Function to draw the path based on intersections
def draw_path(intersections):
    for i in range(len(intersections)):
        x, y = intersections[i]
        robot.goto(x, y)

# Function to simulate robot's movement
def simulate_robot():
    global close_goal_point, min_dis

    while True:
        intersections = [intersection for intersection in close_goal_point if (intersection[0] >= 0.90 and intersection[0] <= 6.0) and abs(intersection[1]) <= 1.5]

        if len(intersections) > 1:
            min_dis = math.sqrt(intersections[0][0]**2 + intersections[0][1]**2)

        if len(intersections) > 0:
            if 0.6 <= intersections[0][0] <= points_max_x:
                steer = cal_steering()
                if 15 < abs(steer) < 25:
                    steer *= 1.2
                steering = max(min(steer, 30), -30)
                robot.setheading(steering)
                robot.forward(2)

def main():
    # Set up the turtle screen
    screen = turtle.Screen()
    screen.setup(800, 600)

    # Call the BoundingBox_callback function to populate close_goal_point
    BoundingBox_callback(None)

    # Draw the path based on intersections
    draw_path(close_goal_point)

    # Simulate the robot's movement
    simulate_robot()

    # Close the turtle graphics window when clicked
    screen.exitonclick()

if __name__ == "__main__":
    main()
