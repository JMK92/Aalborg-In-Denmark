#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor, DistanceSensor

timestep = 64
# create the Robot instance.
robot = Robot()

armMotors = []
armsNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

hand = []
handNames = ['finger_1_joint_1', 'finger_2_joint_1', 'finger_middle_joint_1']


# get the time step of the current world.1
    
while robot.step(timestep) != -1:

    for i in range(6):
        armMotors.append(robot.getDevice(armsNames[i]))
        
    for i in range(3):
        hand.append(robot.getDevice(handNames[i]))
        hand[i].setPosition(0.85)

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
