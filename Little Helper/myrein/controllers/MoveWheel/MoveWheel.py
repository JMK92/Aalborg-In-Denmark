#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7

from controller import Robot, Motor

TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# get the motor devices
#leftbackMotor = robot.getDevice('wheel1')
#rightfrontMotor = robot.getDevice('wheel2')
#rightbackMotor = robot.getDevice('wheel3')
#leftfrontMotor = robot.getDevice('wheel4')
# set the target position of the motors

#left
#leftbackMotor.setPosition(10.0)
#rightfrontMotor.setPosition(10.0)
#rightbackMotor.setPosition(-10.0)
#leftfrontMotor.setPosition(-10.0)


#sleep(1.0);

# right front diagonal
#leftbackMotor.setPosition(0.0)
#rightbackMotor.setPosition(20.0)
#leftfrontMotor.setPosition(20.0)
#rightfrontMotor.setPosition(0.0)
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

cmd = [
  [-2, -2, +2, +2], [-2, -2, 0, 0], [1, -1, -1, 1], [2, 0, 0, 2], [+2, +2, -2, -2], [2, 2, 2, 2]
]

SPEED_FACTOR = 4.0


while robot.step(TIME_STEP) != -1:    
  
  for i in range(4):
      wheels.append(robot.getDevice(wheelsNames[i]))
      wheels[i].setPosition(float('inf'))
      
  for i in range(6):
      for j in range(4):
          wheels[j].setVelocity(cmd[i][j] * SPEED_FACTOR)

      for k in range(100):
          robot.step(8)

  pass
  