#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = 64

armMotors = []
armsNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

cmd = [
  [-1, -1, +1, 0, 0, 0], [2, 2, -2, 0, 0, 0]
]

SPEED_FACTOR = 4.0

while robot.step(TIME_STEP) != -1:    
  
  for i in range(6):
      armMotors.append(robot.getDevice(armsNames[i]))
      armMotors[i].setPosition(float('inf'))
      
  for i in range(2):
      for j in range(6):
          armMotors[j].setVelocity(cmd[i][j] * SPEED_FACTOR)

      for k in range(100):
          robot.step(8)

  pass
  

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
