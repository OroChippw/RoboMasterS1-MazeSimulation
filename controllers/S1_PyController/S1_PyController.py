"""S1_PyController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot , Keyboard

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
def Mecanum_Calculator(Vx , Vy , w , idx):
    motorVelocity = [0 , 0 , 0 , 0]
    motorVelocity[0] = Vy - Vx + w * (Vx + Vy)
    motorVelocity[1] = Vy + Vx - w * (Vx + Vy)
    motorVelocity[2] = Vy - Vx - w * (Vx + Vy)
    motorVelocity[3] = Vy + Vx + w * (Vx + Vy)
    
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
    wheelsSpeeds = []
    wheelsNames = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_br_motor', 'wheel_bl_motor']
    for idx in range(len(wheelsNames)):
        # wheels.append(robot.getMotor(wheelsNames[idx]))
        wheels.append(robot.getDevice(wheelsNames[idx]))
        wheels[idx].setPosition(float('inf'))
        wheels[idx].setVelocity(0.0)
    

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
        changed = False
        
        key = keyboard.getKey()
        if (key == ord('W')):
            Vy = 5
            if(index != 0) : 
                print("[INFO] Go Ahead")
                index = 0
                changed = True
            else :
                changed = False
        if (key == ord('S')):
            Vy = -5
            if(index != 1) : 
                print("[INFO] Back")
                index = 1
                changed = True
            else :
                changed = False
        if (key == ord('D')):
            Vx = 10
            if(index != 2) : 
                print("[INFO] Pan Right")
                index = 2
                changed = True
            else :
                changed = False
        if (key == ord('A')):
            Vx = -10
            if(index != 3) : 
                print("[INFO] Pan Left")
                index = 3
                changed = True
            else :
                changed = False
        if (key==Keyboard.UP):
            pitch_motor.setVelocity(-1.0)
            print('[INFO] Pitch axis raised')
        if (key==Keyboard.DOWN):
            pitch_motor.setVelocity(1.0)
            print('[INFO] Pitch axis down')  
        if (key==Keyboard.LEFT):
            yaw_motor.setVelocity(1.0)
            print('[INFO] YAW axis rotates clockwise')  
        if (key==Keyboard.RIGHT):
            yaw_motor.setVelocity(-1.0)
            print('[INFO] YAW axis rotates counterclockwise') 
        if (key==-1):
            Vx=0
            Vy=0
            w=0
            pitch_motor.setVelocity(0)
            yaw_motor.setVelocity(0)
            if (index != -1):
                print('[INFO] Stop')
                index = -1
                changed = True
            else :
                changed = False
                
        if (changed):
            print(f"w - {w} ; fr - {Mecanum_Calculator(Vx,Vy,w,0)} ; fl - {Mecanum_Calculator(Vx,Vy,w,1)} ; \
                bl - {Mecanum_Calculator(Vx,Vy,w,2)} ; br - {Mecanum_Calculator(Vx,Vy,w,3)}")
        
        wheels[0].setVelocity(Mecanum_Calculator(Vx,Vy,w,0))
        wheels[1].setVelocity(Mecanum_Calculator(Vx,Vy,w,1))
        wheels[2].setVelocity(Mecanum_Calculator(Vx,Vy,w,2))
        wheels[3].setVelocity(Mecanum_Calculator(Vx,Vy,w,3))
        
        
if __name__ == '__main__':
    main()

