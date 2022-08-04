#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7

from controller import Robot, Motor

TIME_STEP = 64

# create the Robot instance.
robot = Robot()

ds = []
dsNames = ['lidar_1_1', 'lidar_2_1']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    #print(ds[i])
    print(ds[i])
    ds[i].enable(TIME_STEP)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0
while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:  # read sensors
         # list to number so can detect
        for i in range(2):            
            if ds[i].getRangeImage() < 0.3:
                avoidObstacleCounter = 100
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)