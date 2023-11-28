"""S1_PyController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot , Keyboard , Display
from math import sin , cos

MAX_SPEED = 15
SENSOR_THRESHOLD = 70 # sensitivity of sensors surrounding robot for obstacle detection

class RobotController():
    def __init__(self , robot , timestep ,max_speed=15 , sensor_threshold=70) -> None:
        # Properties
        self.robot = robot
        self.timestep = timestep
        self.status_changed = False
        
        self.Vx = 0
        self.Vy = 0
        self.w = 0
        
        # Device 、Motors and Sensors
        self.keyboard = None
        self.maincarema = None
        self
        
        # Thresholds
        self.max_speed = max_speed
        self.sensor_threshold = sensor_threshold
        
        self._init_component()
        
    def _stop(self):
        for idx in range(len(wheelsNames)):
            wheels[idx].setVelocity(0.0)
            
        return wheels

    def _instructions_intro(self):
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

    def _init_component(self , maincarema_name='shooter_camera'):
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(1)
        self.maincarema = self.robot.getDevice(maincarema_name)
        self.maincamera.enable(self.timestep)
        pass

# <------ Calculate Individual wheel ------>
Vx = Vy= 0 # linear velocities (Vx and Vy) , Positive are ahead and right
w = 0 # angular velocity (w) , Positive are counterclockwise
# Calculating individual wheel velocities based on linear and angular velocities.
def Mecanum_Calculator(Vx , Vy , w , idx , max_velocity=None):
    motorVelocity = [0 , 0 , 0 , 0]
    R = 1 # Mecanum radius
    distance_x = 0.18
    distance_y = 0.25
    Vx_robot = Vx * cos(w) - Vy * sin(w)
    Vy_robot = Vx * sin(w) + Vy * cos(w)
    motorVelocity[0] = Vy - Vx + w * (R)
    motorVelocity[1] = Vy + Vx - w * (R)
    motorVelocity[2] = Vy - Vx - w * (R)
    motorVelocity[3] = Vy + Vx + w * (R)
    # if idx == 0:
    #     motorVelocity[0] = Vy_robot - Vx_robot + w * (R)
    # elif idx == 1:
    #     motorVelocity[1] = Vy_robot + Vx_robot - w * (R)
    # elif idx == 2:
    #     motorVelocity[2] = Vy_robot - Vx_robot - w * (R)
    # elif idx == 3:
    #     motorVelocity[3] = Vy_robot + Vx_robot + w * (R)
    # print(Vx_robot , Vy_robot , motorVelocity)
    if max_velocity is not None:    
        for i in range(len(motorVelocity)):
            if abs(motorVelocity[i]) > max_velocity:
                motorVelocity[i] = max_velocity if motorVelocity[i] > 0 else -max_velocity
    
    return motorVelocity[idx]

    
def main():
    # <------ Instructions Introduction ------>
    # create the Robot instance.
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print(f"[INFO] Get timestep : {timestep}")
    
    # <------ Init Robot Controller and show Instructions Introduction ------>
    robot_controller = RobotController(robot , MAX_SPEED , SENSOR_THRESHOLD)
    robot_controller._instructions_intro()
    
    # <------ Component Motor/Sensor Mapping ------>
    # Gets the robot's keyboard device and robot's camera device
    keyboard = robot.getKeyboard()
    keyboard.enable(1)
    
    maincamera = robot.getDevice('shooter_camera')
    maincamera.enable(timestep)
    # maincamera.setFocalDistance(0.01) # Set FocalLength
    
    width = maincamera.getWidth()
    height = maincamera.getHeight()
    print(f"[INFO] Main Timestep : {timestep} , Width : {width} , Height : {height}")
    
    # Gets the motors for controlling the yaw and pitch movements
    yaw_motor = robot.getDevice('yaw_motor')
    yaw_motor.setPosition(float('inf'))
    yaw_motor.setVelocity(0.0)
    yaw_sensor = robot.getDevice('yaw_sensor')
    yaw_sensor.enable(timestep)

    target_pitch_angle = 0.0
    pitch_motor = robot.getDevice('pitch_motor')
    pitch_motor.setPosition(float('inf'))
    pitch_motor.setVelocity(0.0)
    pitch_sensor = robot.getDevice("pitch_sensor")
    pitch_sensor.enable(timestep)

    '''
        Gets the motors for controlling the wheels
        # 0-------2
        #     |
        #     |
        #     |
        # 1-------3
        [0-fl-front left] [1-bl-back left] [2-fr-front right] [3-br-back left] 
    '''
    wheels_motors = []
    wheels_sensors = []
    wheelsMaxSpeeds = []
    wheels_MotorNames = ['wheel_fl_motor', 'wheel_bl_motor', 'wheel_fr_motor', 'wheel_br_motor']
    wheels_SensorNames = ['wheel_fl_sensor', 'wheel_bl_sensor', 'wheel_fr_sensor', 'wheel_br_sensor']
    assert len(wheels_MotorNames) == len(wheels_SensorNames) , \
        f"[ERROR] The number of motors and sensors of the transmission wheel should be same"
        
    for idx in range(len(wheels_MotorNames)):
        # wheels.append(robot.getMotor(wheelsNames[idx]))
        wheels_motors.append(robot.getDevice(wheels_MotorNames[idx]))
        wheels_sensors.append(robot.getDevice(wheels_SensorNames[idx]))
        
        wheelsMaxSpeeds.append(wheels_motors[idx].getMaxVelocity())
        wheels_motors[idx].setPosition(float('inf'))
        wheels_motors[idx].setVelocity(0.0)
    print(f"[INFO] wheelsMaxSpeeds : {wheelsMaxSpeeds}")
    
    # <------ Component Motor/Sensor Mapping ------>
    
    
    # <------ Main loop ------>
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Get sensors outputs
        sensor_values = []
        avoidObstacleCounter = 0
        index = -1
    
        key = keyboard.getKey()
        if (key == ord('W')):
            Vx = Vy = 5
            index = 0
        elif (key == ord('S')):
            Vx = Vy = -5
            index = 1
        elif (key == ord('D')):
            Vx = 5
            Vy = -5
            index = 2    
            changed = True
            
        elif (key == ord('A')):
            Vx = -5
            Vy = 5
            index = 3
            changed = True
               
        
        # elif (key == ord('Q')):
        #     w = 1.0
        #     if (index != 4):
        #         index = 4
        #         print("[INFO] Rotate counterclockwise")
        #         changed = True
        #     else :
        #         changed = False
        # elif (key == ord('E')):
        #     w = -1.0
        #     if (index != 5):
        #         index = 5
        #         print("[INFO] Rotate clockwise")
        #         changed = True
        #     else :
        #         changed = False
        elif (key==Keyboard.UP):
            pitch_motor.setVelocity(-1.0)
            current_pitch_angle = pitch_sensor.getValue()
            print(f"[INFO] Pitch axis raised , Current pitch angle of the pitch_motor is {current_pitch_angle}")
        elif (key==Keyboard.DOWN):
            pitch_motor.setVelocity(1.0)
            current_pitch_angle = pitch_sensor.getValue()
            print(f"[INFO] Pitch axis down , Current pitch angle of the pitch_motor is {current_pitch_angle}")
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
            wheels_motors[0].setVelocity(0)
            wheels_motors[1].setVelocity(0)
            wheels_motors[2].setVelocity(0)
            wheels_motors[3].setVelocity(0)
            index = -1

        
        if (index == 0 or index == 1):
            wheels_motors[0].setVelocity(Vx)
            wheels_motors[1].setVelocity(Vx)
            wheels_motors[2].setVelocity(Vy)
            wheels_motors[3].setVelocity(Vy)
            # if changed:
            #     print(f"[INFO] fl:{Vx} ; bl:{Vx} ; fr :{Vy} ; br : {Vy}")
        elif (index == 2 or index == 3):
            wheels_motors[0].setVelocity(Vx)
            wheels_motors[1].setVelocity(Vy)
            wheels_motors[2].setVelocity(Vy)
            wheels_motors[3].setVelocity(Vx)
            # if changed:
            #     print(f"[INFO] fl:{Vy} ; bl:{Vx} ; fr :{Vx} ; br : {Vy}")
            #     print(f"[INFO] fl:{Vx} ; bl:{Vy} ; fr :{Vy} ; br : {Vx}")
                
       
        
        
        
        
if __name__ == '__main__':
    main()

