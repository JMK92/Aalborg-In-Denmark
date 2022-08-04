#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robot(robot):
    TIME_STEP = 32
    max_speed = 6.28
    
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
    for i in range(4):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)
    # Motors
    #leftbackMotor = robot.getDevice('wheel1')
    #rightbackMotor = robot.getDevice('wheel2')
    #leftfrontMotor = robot.getDevice('wheel3')
    #rightfrontMotor = robot.getDevice('wheel4')    
    
    #leftbackMotor.setPosition(float('inf'))
    #rightbackMotor.setPosition(float('inf'))
    #leftfrontMotor.setPosition(float('inf'))
    #rightfrontMotor.setPosition(float('inf'))   
    
    #leftbackMotor.setVelocity(0.0)
    #rightbackMotor.setVelocity(0.0)
    #leftfrontMotor.setVelocity(0.0)
    #rightfrontMotor.setVelocity(0.0)  
    
    # Lidar
    #ls = [] 
    #lsNames = ['lidar_1_1', 'lidar_2_1']
    #for i in range(2):
    #    ls.append(robot.getDevice(lsNames[i]))
   
     #   print(ls[i])
     #   ls[i].enable(TIME_STEP)
        #ds[i].enablePoinCloud()
    
    lidar = robot.getDevice('lidar_1_1')
    lidar_2 = robot.getDevice('lidar_2_1')
    
    lidar.enable(TIME_STEP)
    lidar_2.enable(TIME_STEP)
    
    leftbackMotor = robot.getDevice('wheel1')
    rightfrontMotor = robot.getDevice('wheel2')
    rightbackMotor = robot.getDevice('wheel3')
    leftfrontMotor = robot.getDevice('wheel4')
    
    lidar.enablePointCloud()
    lidar_2.enablePointCloud()
    #lidar_2.enablePointCloud()
    #lidar.enablePointCloud()
    
    while robot.step(TIME_STEP) != -1:
    
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        rangeImage_2 = lidar_2.getRangeImage();
        lidarPoints = lidar.getPointCloud();
    # Print the first 10 values of the range image.
    # The range image stores the distances from left to right, from first to last layer
        for i in range(10):
            print(str(rangeImage[i]) + " ", end='')
        
        print('')       
        
        for j in range(10):
            print(str(rangeImage_2[j]) + " ", end='')
        
        print('')
        print("x: " + str(lidarPoints[0].x) + " y: " + str(lidarPoints[0].y) + " z: " + str(lidarPoints[0].z) + " layer: " + str(lidarPoints[0].layer_id))                    
        
        leftbackMotor.setVelocity(max_speed*0.25)
        rightbackMotor.setVelocity(max_speed*0.25)
        leftfrontMotor.setVelocity(max_speed*0.25)
        rightfrontMotor.setVelocity(max_speed*0.25)  
  
    
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
   
    


