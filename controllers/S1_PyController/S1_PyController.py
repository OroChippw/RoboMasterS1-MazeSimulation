"""S1_PyController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot , Keyboard
from math import sin , cos

MAX_SPEED = 15
SENSOR_THRESHOLD = 70 # sensitivity of sensors surrounding robot for obstacle detection
MAX_PITCH_ANGLE = 60

def Instructions_Intro():
    '''
        Prints instructions for controlling the robot using the keyboard and mouse.
    '''
    print("<------ [DEMO] RoboMaster S1 Maze Simulation ------>")
    
    print("<------ [START] instructions for controlling the robot [START] ------>")
    print("[INFO] Please use keyboard to control RoboMaster S1")
    print("[INFO] [W - Go Ahead] [S - Back] [A - Pan Left] [D - Pan Right]")
    print("[INFO] [→ - YAW axis rotates counterclockwise]")
    print("[INFO] [← - YAW axis rotates clockwise]")
    print("[INFO] [↑ - Pitch axis raised]")
    print("[INFO] [↓ - Pitch axis down]")
    print("<------ [END] instructions for controlling the robot [END] ------>")
    
    return


def stop(wheelsNames , wheels):
    for idx in range(len(wheelsNames)):
        wheels[idx].setVelocity(0.0)
    
    return wheels

# <------ Calculate Individual wheel ------>
Vx = Vy= 0 # linear velocities (Vx and Vy) , Positive are ahead and right
w = 0 # angular velocity (w) , Positive are counterclockwise
# Calculating individual wheel velocities based on linear and angular velocities.
def Mecanum_Calculator(Vx , Vy , w , idx , max_velocity=None):
    motorVelocity = [0 , 0 , 0 , 0]
    R = 1 # Mecanum radius
    Vx_robot = Vx * cos(w) - Vy * sin(w)
    Vy_robot = Vx * sin(w) + Vy * cos(w)
    # motorVelocity[0] = Vy - Vx + w * (R)
    # motorVelocity[1] = Vy + Vx - w * (R)
    # motorVelocity[2] = Vy - Vx - w * (R)
    # motorVelocity[3] = Vy + Vx + w * (R)
    if idx == 0:
        motorVelocity[0] = Vy_robot - Vx_robot + w * (R)
    elif idx == 1:
        motorVelocity[1] = Vy_robot + Vx_robot - w * (R)
    elif idx == 2:
        motorVelocity[2] = Vy_robot - Vx_robot - w * (R)
    elif idx == 3:
        motorVelocity[3] = Vy_robot + Vx_robot + w * (R)
    # print(Vx_robot , Vy_robot , motorVelocity)
    if max_velocity is not None:    
        for i in range(len(motorVelocity)):
            if abs(motorVelocity[i]) > max_velocity:
                motorVelocity[i] = max_velocity if motorVelocity[i] > 0 else -max_velocity
    
    return motorVelocity[idx]

# <------ Distance Sensor Settings ------>
# distance_sensor_list = []
# dsNames = ['ds_left' , 'ds_right']
# for idx in range(len(dsNames)):
#     if robot.getDevice(dsNames[idx]) :
#         # distance_sensor_list.append(robot.getDistanceSensor(dsNames[idx]))
#         distance_sensor_list.append(robot.getDevice(dsNames[idx]))
#         distance_sensor_list[idx].enable(timestep)
#     else :
#         print("[WARNINGS] Can't get any distance sensors")


def main():
    # <------ Instructions Introduction ------>
    Instructions_Intro()
    
    # <------ Instructions Introduction ------>
    # create the Robot instance.
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print(f"[INFO] Get timestep : {timestep}")
    
    # <------ Component Motor Mapping ------>
    # Gets the robot's keyboard device and robot's camera device
    keyboard = robot.getKeyboard()
    keyboard.enable(1)
    
    maincamera = robot.getDevice('camera')
    maincamera.enable(timestep)
    width = maincamera.getWidth()
    height = maincamera.getHeight()
    print(f"[INFO] Main Timestep : {timestep} , Width : {width} , Height : {height}")
    
    # Gets the motors for controlling the yaw and pitch movements
    yaw_motor = robot.getDevice('yaw_motor')
    yaw_motor.setPosition(float('inf'))
    yaw_motor.setVelocity(0.0)

    pitch_motor = robot.getDevice('pitch_motor')
    pitch_motor.setPosition(float('inf'))
    pitch_motor.setVelocity(0.0)

    '''
        Gets the motors for controlling the wheels
        # 1-------0
        #     |
        #     |
        #     |
        # 2-------3
        [0-fr-front right] [1-fl-front left] [2-bl-back left] [3-br-back left] 
    '''
    wheels = []
    wheels_sensors = []
    wheelsMaxSpeeds = []
    wheelsMotorNames = ['wheel_fr_motor', 'wheel_fl_motor', 'wheel_bl_motor', 'wheel_br_motor']
    wheels_SensorNames = ['wheel_fr_sensor', 'wheel_fl_sensor', 'wheel_bl_sensor', 'wheel_br_sensor']
    
    for idx in range(len(wheelsMotorNames)):
        # wheels.append(robot.getMotor(wheelsNames[idx]))
        wheels.append(robot.getDevice(wheelsMotorNames[idx]))
        wheels_sensors.append(robot.getDevice(wheels_SensorNames[idx]))
        
        wheelsMaxSpeeds.append(wheels[idx].getMaxVelocity())
        wheels[idx].setPosition(float('inf'))
        wheels[idx].setVelocity(0.0)
    print(f"wheelsMaxSpeeds : {wheelsMaxSpeeds}")
    
    changed = False
    
    # <------ Main loop ------>
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Get sensors outputs
        dsValues = []
        # for idx in range(len(dsNames)):
        #     dsValues.append(distance_sensor_list[idx].getValue())
        # left_obstacle = dsValues[0] > OBSTACLE_THESHOLD
        # right_obstacle = dsValues[1] > OBSTACLE_THESHOLD
        # if left_obstacle :
        #     print("[INFO] There is an obstacle on the left")
        # if right_obstacle :
        #     print("[INFO] There is an obstacle on the right")

        avoidObstacleCounter = 0    
        index = -1
    
        key = keyboard.getKey()
        if (key == ord('W')):
            Vy = 5
            if(index != 0) : 
                print("[INFO] Go Ahead")
                index = 0
                changed = True
            else :
                changed = False
        elif (key == ord('S')):
            Vy = -5
            if(index != 1) : 
                print("[INFO] Back")
                index = 1
                changed = True
            else :
                changed = False
        elif (key == ord('D')):
            Vx = 5
            Vy = 0.0
            w = 0
            if(index != 2) : 
                print("[INFO] Pan Right")
                index = 2
                changed = True
            else :
                changed = False
        elif (key == ord('A')):
            Vx = -5
            Vy = 0.0
            w = 0
            if(index != 3) : 
                print("[INFO] Pan Left")
                index = 3
                changed = True
            else :
                changed = False
                
        elif (key == ord('Q')):
            w = 1.0
            if (index != 4):
                index = 4
                print("[INFO] Rotate counterclockwise")
                changed = True
            else :
                changed = False
        elif (key == ord('E')):
            w = -1.0
            if (index != 5):
                index = 5
                print("[INFO] Rotate clockwise")
                changed = True
            else :
                changed = False
        elif (key==Keyboard.UP):
            pitch_motor.setVelocity(-1.0)
            print('[INFO] Pitch axis raised')
        elif (key==Keyboard.DOWN):
            pitch_motor.setVelocity(1.0)
            print('[INFO] Pitch axis down')  
        elif (key==Keyboard.LEFT):
            yaw_motor.setVelocity(1.0)
            print('[INFO] YAW axis rotates clockwise')  
        elif (key==Keyboard.RIGHT):
            yaw_motor.setVelocity(-1.0)
            print('[INFO] YAW axis rotates counterclockwise') 
        elif (key==-1):
            Vy, Vx, w = 0, 0, 0
            pitch_motor.setVelocity(0)
            yaw_motor.setVelocity(0)
            if (index != -1):
                print('[INFO] Stop')
                index = -1
                changed = True
            else :
                changed = False
                
        if (changed):
            print(f"w:{w} ; fr:{Mecanum_Calculator(Vx,Vy,w,0,wheelsMaxSpeeds[0])} ; fl:{Mecanum_Calculator(Vx,Vy,w,1,wheelsMaxSpeeds[1])}")
            print(f"bl:{Mecanum_Calculator(Vx,Vy,w,2,wheelsMaxSpeeds[2])} ; br:{Mecanum_Calculator(Vx,Vy,w,3,wheelsMaxSpeeds[3])}")
        
        wheels[0].setVelocity(Mecanum_Calculator(Vx,Vy,w,0,wheelsMaxSpeeds[0]))
        wheels[1].setVelocity(Mecanum_Calculator(Vx,Vy,w,1,wheelsMaxSpeeds[1]))
        wheels[2].setVelocity(Mecanum_Calculator(Vx,Vy,w,2,wheelsMaxSpeeds[2]))
        wheels[3].setVelocity(Mecanum_Calculator(Vx,Vy,w,3,wheelsMaxSpeeds[3]))
        
        
if __name__ == '__main__':
    main()

