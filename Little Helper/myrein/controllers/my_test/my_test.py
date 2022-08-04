#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7
import math
#import gripper
from controller import Robot, Motor
import numpy as np
import time

TIME_STEP = 64
# create the Robot instance.
robot = Robot()
# get the motor devices

#leftbackMotor = robot.getDevice('wheel1')
#rightfrontMotor = robot.getDevice('wheel2')
#rightbackMotor = robot.getDevice('wheel3')
#leftfrontMotor = robot.getDevice('wheel4')

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
speed = 10

################### Initialize Constants ##################
# Set the robot vehicle's dimensions
# Robot is 2 feet in width and 4 feet in length
ROBOT_WIDTH = 0.2
ROBOT_LENGTH = 0.4
 
# Maximum speed of the robot in feet/second
MAX_SPEED = 15.0
 
# Robot vehicle's wheel radius in feet
WHEEL_RADIUS = 0.05


radius = 0.05
height = 0.02
################### Global Variables ######################
# The robot vehicle object (which will be a rectangle)
rect = None
 
# The coordinates of the vehicle reference point
x_pos_ref = None # in feet
y_pos_ref = None # in feet
vehicle_angle_ref = None # in degrees
 
def convert_local_velocity_to_global(v_x_local, v_y_local):
    """
    Convert the velocities in the x and y-directions in 
    the local reference frame to velocities in the global
    reference frame.
    @param v_x_local float: Velocity in the x-direction 
        in the local reference frame in ft/s
    @param v_y_local float: Velocity in the y-direction 
        in the local reference frame in ft/s
    @return v_x_global float: Velocity in the x-direction 
        in the global reference frame in ft/s
    @return v_y_global float: Velocity in the y-direction 
        in the global reference frame in ft/s
    """
    v_x_global = (v_x_local * math.cos(math.radians(
        vehicle_angle_ref))) - (v_y_local * math.sin(
        math.radians(vehicle_angle_ref)))
 
    v_y_global = (v_x_local * math.sin(math.radians(
        vehicle_angle_ref))) + (v_y_local * math.cos(
        math.radians(vehicle_angle_ref)))
 
    return v_x_global, v_y_global 

def get_distance_btw_points(x1,y1,x2,y2):
    """
    Calculate the distance between two points in feet.
    @param x1: The x-coordinate of point 1
    @param y1: The y-coordinate of point 1
    @param x2: The x-coordinate of point 2
    @param y2: The y-coordinate of point 2
    @return distance: float: in feet
    """
    distance = math.sqrt((x2 - x1)**2 + (y2 -y1)**2)
    return distance
 
def get_robot_motion_values(rot1, rot2, rot3, rot4):
    """
    Calculate the velocities in the x and y-directions in 
    the local reference frame as well as the rotation rate
    of the vehicle reference point.
    @param rot1: Rotation rate of wheel 1 in degrees/sec
    @param rot2: Rotation rate of wheel 2 in degrees/sec
    @param rot3: Rotation rate of wheel 3 in degrees/sec
    @param rot4: Rotation rate of wheel 4 in degrees/sec
    @return v_x_local float: Velocity in the x-direction 
        in the local reference frame in ft/s
    @return v_y_local float: Velocity in the y-direction 
        in the local reference frame in ft/s
    @return rotation_rate: Rotation rate of the vehicle
        reference point in degrees per second in the 
        counter-clockwise direction
    """
    v_x_local = (WHEEL_RADIUS/4) * (
        rot1 - rot2 - rot3 + rot4)
    v_y_local = (WHEEL_RADIUS/4) * (
        rot1 + rot2 + rot3 + rot4)
    rotation_rate = (WHEEL_RADIUS/(4*(
        ROBOT_LENGTH + ROBOT_WIDTH)))*(
        -rot1 + rot2 - rot3 + rot4)
 
    return v_x_local, v_y_local, rotation_rate
 
def get_wheel_rot_rates(v_x_local, v_y_local, 
        rotation_rate):
    """
    Calculate the wheel rotation rates.
    @param v_x_local: Velocity in the x-direction 
        in the local reference frame in ft/s
    @param v_y_local: Velocity in the y-direction 
        in the local reference frame in ft/s
    @param rotation_rate: Rotation rate in degrees per 
        second in the counter-clockwise direction (vehicle
        reference point)
    @return rot1, rot2, rot3, rot4: float: Wheel rotation 
        rates in degrees per second
    """
    rot1 = (1/WHEEL_RADIUS)*(v_y_local + v_x_local - (
        ROBOT_LENGTH + ROBOT_WIDTH) * rotation_rate)
    rot2 = (1/WHEEL_RADIUS)*(v_y_local - v_x_local + (
        ROBOT_LENGTH + ROBOT_WIDTH) * rotation_rate)
    rot3 = (1/WHEEL_RADIUS)*(v_y_local - v_x_local - (
        ROBOT_LENGTH + ROBOT_WIDTH) * rotation_rate)
    rot4 = (1/WHEEL_RADIUS)*(v_y_local + v_x_local + (
        ROBOT_LENGTH + ROBOT_WIDTH) * rotation_rate)
 
    return rot1, rot2, rot3, rot4
 
def get_speed(distance, time):
    """
    Calculate the speed of the vehicle using the distance 
    and time.
    @param distance: The distance the robot vehicle must 
        travel between 2 or more points in feet
    @param time: Total travel time in seconds
    @return speed: float
    """
    speed = distance / time
    return speed
 
def get_velocity_magnitude(velocity_x, velocity_y):
    """
    Calculate the speed of the vehicle using the x and y
    components of the velocity.
    @param velocity_x: Velocity in x-direction in ft/sec
    @param velocity_y: Velocity in y-direction in ft/sec
    @return speed: float
    """
    speed = math.sqrt(((velocity_x)**2) + ((velocity_y)**2))
    return speed

def point_execution_with_wp(x_wp1 = 5.0, y_wp1 = 5.0, 
        x_wp2 = 5.0, y_wp2 = -5.0, 
        x_wp3 = -5.0, y_wp3 = 5.0,
        x_final = -5.0, y_final = -5.0, 
        orientation_final = 90.0, deadline = 5.0):
    """
    The user can specify a straight line path to an
    end point, including a desired end orientation.
    Waypoints between start and endpoints can be specified.
    @param x_wp1: x-coordinate of waypoint 1
    @param y_wp1: y-coordinate of waypoint 1
    @param x_wp2: x-coordinate of waypoint 2
    @param y_wp2: y-coordinate of waypoint 2
    @param x_wp3: x-coordinate of waypoint 3
    @param y_wp3: y-coordinate of waypoint 3
    @param x_final: x-coordinate of end point
    @param y_final: y-coordinate of end point
    @param orientation_final: Desired end orientation in 
        degrees.
    @param deadline float: Time taken to get from current 
        point to end point in seconds.
    """
    global hist_x_vals, hist_y_vals
    global x_pos_ref, y_pos_ref, vehicle_angle_ref
 
    # List of all x and y values along the path
    x_list = [x_pos_ref, x_wp1, x_wp2, x_wp3, x_final]
    y_list = [y_pos_ref, y_wp1, y_wp2, y_wp3, y_final]
 
    # Number of legs on the path
    no_of_legs = len(x_list) - 1
 
    # Keep track of the distances of each leg of the trip
    distance_list = []
 
    # Calculate the distance of each leg of the trip in ft
    for idx in range(no_of_legs):
        distance_list.append(get_distance_btw_points( x_list[idx],y_list[idx],x_list[idx + 1],y_list[idx + 1]))
 
    total_distance = sum(distance_list)
 
    speed = get_speed(total_distance,deadline)
 
    # Method will not run if speed entered is >15 ft/s
    if (is_too_fast(speed)):
        print("Error: Maximum speed is 15 " +
            "ft/s.\nPlease increase the deadline.\n\n" +
            "Speed = " + str(speed) + " ft/s")
        
        stop()
 
    # Calculate rotation rate in degrees/sec
    rotation_rate = (orientation_final - (
        vehicle_angle_ref))/(deadline)
 
    # Calculate the duration of each leg in seconds
    deadline_list = []
 
    for idx in range(no_of_legs):
        deadline_list.append((distance_list[
            idx]/total_distance) * deadline)
 
    # Number of frames per leg
    no_of_frames_per_leg = 5
 
    # Time intervals in seconds for each leg
    dt_list = []
 
    for idx in range(no_of_legs):
        dt_list.append(deadline_list[idx]/(
            no_of_frames_per_leg))
 
    # Move through each leg of the trip
    for idx in range(no_of_legs):
 
        # Number of frames per leg
        no_of_frames_per_leg = 5
 
        # Calculate the direction of travel of the vehicle 
        # reference point in degrees
        direction_global = math.degrees(math.atan2(
            (y_list[idx + 1] - y_list[idx]),(x_list[
            idx + 1] - x_list[idx])))
 
        # Calculate velocity in the x-direction in the
        # global reference frame
        v_x_global = (speed) * math.cos(
            math.radians(direction_global))
 
        # Calculate velocity in the y-direction in the
        # global reference frame
        v_y_global = (speed) * math.sin(
            math.radians(direction_global))
 
        for num in range(no_of_frames_per_leg):
 
            x_initial = x_pos_ref
            y_initial = y_pos_ref
            vehicle_angle_ref_initial = vehicle_angle_ref
 
            # Determine the new x-coordinate of the vehicle
            # reference point
            x_pos_ref = x_initial + (v_x_global * dt_list[
                idx])
 
            # Determine the new y-coordinate of the vehicle
            # reference point
            y_pos_ref = y_initial + (v_y_global * dt_list[
                idx])
 
            # Determine the new orientation of the vehicle
            # reference point
            vehicle_angle_ref = (
                vehicle_angle_ref_initial + (
                rotation_rate * dt_list[idx]))
 
            # Reposition grid if we are close to the edge
            if (is_close_to_edge(x_pos_ref, y_pos_ref)):
                plot_grid(x_pos_ref, y_pos_ref)
 
            # Move robot to new position
            plot_robot(x_pos_ref,y_pos_ref,math.radians(
                vehicle_angle_ref))
            plot_arrow(x_pos_ref,y_pos_ref,math.radians(
                vehicle_angle_ref))
 
            # Update path traveled by robot
            hist_x_vals.append(x_pos_ref)
            hist_y_vals.append(y_pos_ref)
            plot_path_traveled(hist_x_vals, hist_y_vals)
 
            # Convert direction (global) into direction (local)
            direction = direction_global - vehicle_angle_ref
 
            # Calculate velocity in the x-direction in the
            # local reference frame
            v_x_local = (speed) * math.cos(
                math.radians(direction))
 
            # Calculate velocity in the y-direction in the
            # local reference frame
            v_y_local = (speed) * math.sin(
                math.radians(direction))
 
            # Update wheel rotation rates
            rot1, rot2, rot3, rot4 = get_wheel_rot_rates(
                v_x_local, v_y_local, rotation_rate)
 
            # Vehicle position display
            print("VEHICLE POSITION")
            print("X: " + str(x_pos_ref) + " feet")
            print("Y: " + str(y_pos_ref) + " feet")
            print("Orientation: " + str(vehicle_angle_ref) +
                          " degrees\n")
 
            # Wheel rate display
            print("WHEEL ROTATION RATES")
            print("Wheel 1: " + str(rot1) + " degrees/s")
            print("Wheel 2: " + str(rot2) + " degrees/s")
            print("Wheel 3: " + str(rot3) + " degrees/s")
            print("Wheel 4: " + str(rot4) + " degrees/s\n")
 
            # Robot velocity display
            print("ROBOT VELOCITY (LOCAL)")
            print("V_X: " + str(v_x_local) + " ft/s")
            print("V_Y: " + str(v_y_local) + " ft/s\n\n")
 



for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

def forward():
    for i in range(4):
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(speed)
def backward():
    for i in range(4):
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(-speed)
def left():
    for i in range(4):
        wheels[i].setPosition(float('inf'))
        wheels[0].setVelocity(+speed)
        wheels[1].setVelocity(+speed)
        wheels[2].setVelocity(-speed)
        wheels[3].setVelocity(-speed)
def right():
    for i in range(4):
        wheels[i].setPosition(float('inf'))
        wheels[0].setVelocity(-speed)
        wheels[1].setVelocity(-speed)
        wheels[2].setVelocity(+speed)
        wheels[3].setVelocity(+speed)
def stop():
    for i in range(4):
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)



while robot.step(TIME_STEP) != -1:
   #forward()
   #time.sleep(3)
   #stop()
   #backward()
   #time.sleep(3.0)
   #left()
   #right()
   #wb_robot_init();
   #gripper_release();
   get_robot_motion_values(10, 10, -10, -10)
   point_execution_with_wp(x_wp1 = 5.0, y_wp1 = 5.0, 
         x_wp2 = 5.0, y_wp2 = -5.0, 
         x_wp3 = -5.0, y_wp3 = 5.0,
         x_final = -5.0, y_final = -5.0, 
         orientation_final = 90.0, deadline = 5.0)
   pass
       
#if __name__ == "__main__":
    
    #forward()
        