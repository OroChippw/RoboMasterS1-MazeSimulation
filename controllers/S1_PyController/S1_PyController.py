"""
    File : S1_PyController.py
    Author : OroChippw
    Date : 2023.11.29
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot , Keyboard , Display
from math import sin , cos

MAX_SPEED = 15
SENSOR_THRESHOLD = 70 # sensitivity of sensors surrounding robot for obstacle detection

class RobotController():
    '''
        The RobotController of RoboMaster S1
        # 0-------2
        #     |
        #     |
        #     |
        # 1-------3
        [0-fl-front left] [1-bl-back left] [2-fr-front right] [3-br-back right] 
    '''
    def __init__(self , robot , mode=None , timestep=32 ,max_speed=15 , 
                 sensor_threshold=70) -> None:
        # <------ Properties ------>
        self.robot = robot
        self.robot_name = "RoboMaster S1"
        self.timestep = timestep
        self.status = None
        self.temp = None
        self.statusChanged = False
        self.mode = mode
        self.wheels_MotorNames = ['wheel_fl_motor', 
            'wheel_bl_motor', 'wheel_fr_motor', 'wheel_br_motor']
        self.wheels_SensorNames = ['wheel_fl_sensor', 
            'wheel_bl_sensor', 'wheel_fr_sensor', 'wheel_br_sensor']
        assert len(self.wheels_MotorNames) == len(self.wheels_SensorNames) , \
        f"[ERROR] The number of motors and sensors of the transmission wheel should be same"
        
        # linear velocities (Vx and Vy) , Positive are ahead and right
        self.Vx = 0 
        self.Vy = 0
        # angular velocity (w) , Positive are counterclockwise
        self.w = 0
        self.current_key = None # Capture keyboard signals
        self.current_pitch_angle = None
        self.avoidObstacleCounter = 0
        
        # <------ Device 、Motors and Sensors ------>
        self.keyboard = None
        self.maincarema = None
        self.wheels_Motors = []
        self.wheels_Sensors = []
        self.wheels_MaxSpeeds = []
        self.wheels_SensorsValues = []
        self.yaw_motor = None
        self.pitch_motor = None
        
        # <------ Thresholds ------>
        self.max_speed = max_speed
        self.sensor_threshold = sensor_threshold
        
        self._init_component()
        
    def _instructions_intro(self):
        '''
            Func:
                Prints instructions for controlling the robot using the keyboard and mouse.
            Args/Return:
                None
        '''
        print("<------ [DEMO] RoboMaster S1 Maze Simulation ------>")
        print("[INFO] # 0-------2")
        print("[INFO] #     |")
        print("[INFO] #     |")
        print("[INFO] #     |")
        print("[INFO] # 1-------3")
        print("[INFO] [0-fl-front left] [1-bl-back left] [2-fr-front right] [3-br-back right] ")
        print("<------ [START] instructions for controlling the robot [START] ------>")
        print("[INFO] Please use keyboard to control RoboMaster S1")
        print("[INFO] [W - Forward] [S - Backward] [A - Pan Left] [D - Pan Right]")
        print("[INFO] [→ - YAW axis rotates counterclockwise]")
        print("[INFO] [← - YAW axis rotates clockwise]")
        print("[INFO] [↑ - Pitch axis raised]")
        print("[INFO] [↓ - Pitch axis down]")
        print("<------ [END] instructions for controlling the robot [END] ------>")
        
        return
    
    def _reset_Velocity(self):
        self.pitch_motor.setVelocity(0)
        self.yaw_motor.setVelocity(0)
        for idx in range(len(self.wheels_Motors)):
            self.wheels_Motors[idx].setVelocity(0)
        self.status = "Stop"
        
        return 
    
    def _get_Velocity(self):
        return self.Vx , self.Vy , self.w
    
    def _set_max_speed(self , speed):
        self.max_speed = speed
    
    def _get_Image(self):
        return self.maincarema.getImage()
        
    def _init_component(self , maincarema_name='shooter_camera'):
        '''
            Func:
                Complete the initialization of model components (motors, sensors, etc.)
            Args:
                maincarema_name : default : "shooter_camera" 
            Return:
                None
        '''
        try:
            self.timestep = int(self.robot.getBasicTimeStep())
            print(f"[INFO] Get Main timestep : {self.timestep}")
            
            # Gets the robot's keyboard device and robot's camera device
            self.keyboard = self.robot.getKeyboard()
            self.keyboard.enable(1)
            self.maincarema = self.robot.getDevice(maincarema_name)
            self.maincarema.enable(self.timestep)
            self.width = self.maincarema.getWidth()
            self.height = self.maincarema.getHeight()
            print(f"[INFO] Width : {self.width} , Height : {self.height}")
            
            
            # Gets the motors for controlling the yaw and pitch movements
            self.yaw_motor = self.robot.getDevice('yaw_motor')
            self.yaw_motor.setPosition(float('inf'))
            self.yaw_motor.setVelocity(0.0)
            self.yaw_sensor = self.robot.getDevice('yaw_sensor')
            self.yaw_sensor.enable(self.timestep)
            
            self.pitch_motor = self.robot.getDevice('pitch_motor')
            self.pitch_motor.setPosition(float('inf'))
            self.pitch_motor.setVelocity(0.0)
            self.pitch_sensor = self.robot.getDevice("pitch_sensor")
            self.pitch_sensor.enable(self.timestep)        
        
            # Gets the motors for controlling the wheels
            for idx in range(len(self.wheels_MotorNames)):
                self.wheels_Motors.append(self.robot.getDevice(self.wheels_MotorNames[idx]))
                self.wheels_Sensors.append(self.robot.getDevice(self.wheels_SensorNames[idx]))
                
                self.wheels_MaxSpeeds.append(self.wheels_Motors[idx].getMaxVelocity())
                
                self.wheels_Motors[idx].setPosition(float('inf'))
                self.wheels_Motors[idx].setVelocity(0.0)
                
                self.wheels_Sensors[idx].enable(self.timestep)
                
            print(f"[INFO] Complete initialization of sensors and motors")
            print(f"[INFO] wheelsMaxSpeeds : {self.wheels_MaxSpeeds}")
            
        except Exception as e:
            print("[ERROR] ", e)
        
        return 
    
    def _keyboard_catcher(self , key=None):
        '''
            Func:
                Capture/simulate keyboard input signals
            Args:
                key : When key is not None, expect to receive ASCII code to simulate keyboard input. default : None
            Return:
                None
        '''
        if key is not None:
            self.current_key = key
        else:
            self.current_key = self.keyboard.getKey()
        
        if (self.current_key == ord('W')):
            self.Vx = self.Vy = 5
            self.status = "Forward"
        elif (self.current_key == ord('S')):
            self.Vx = self.Vy = -5
            self.status = "Backward"
        elif (self.current_key == ord('D')):
            self.Vx = -5
            self.Vy = 5
            self.status = "Pan Right"
        elif (self.current_key == ord('A')):
            self.Vx = 5
            self.Vy = -5
            self.status = "Pan Left"
        # elif (self.current_key == ord("Q")):
        #     self.w = 1.0
        #     self.status = "Rotate Counterclockwise"
        # elif (self.current_key == ord('E')):
        #     self.w = -1.0
        #     self.status = "Rotate Clockwise"
        elif (self.current_key == Keyboard.UP):
            self.pitch_motor.setVelocity(-1.0)
            self.current_pitch_angle = self.pitch_sensor.getValue()
            self.status = "Pitch Raised"
        elif (self.current_key == Keyboard.DOWN):
            self.pitch_motor.setVelocity(1.0)
            self.current_pitch_angle = self.pitch_sensor.getValue()
            self.status = "Pitch Down"
        elif (self.current_key == Keyboard.LEFT):
            self.yaw_motor.setVelocity(1.0)
            self.status = "Yaw Clockwise"
        elif (self.current_key == Keyboard.RIGHT):
            self.yaw_motor.setVelocity(-1.0)
            self.status = "Yaw CounterClockwise"
        elif (self.current_key == -1):
            self.Vy, self.Vx, self.w = 0, 0, 0
            self._reset_Velocity()
        
        if self.status != self.temp:
            self.statusChanged = True
        
        self.temp = self.status

    def _set_Mecanum_Velocoty(self):
        if (self.status == "Forward" or self.status == "Backward"):
            self.wheels_Motors[0].setVelocity(self.Vx)
            self.wheels_Motors[1].setVelocity(self.Vx)
            self.wheels_Motors[2].setVelocity(self.Vy)
            self.wheels_Motors[3].setVelocity(self.Vy)
            if self.statusChanged:
                print(f"[INFO] fl:{self.Vx} ; bl:{self.Vx} ; fr :{self.Vy} ; br : {self.Vy}")
        elif (self.status == "Pan Left" or self.status == "Pan Right"):
            self.wheels_Motors[0].setVelocity(self.Vx)
            self.wheels_Motors[1].setVelocity(self.Vy)
            self.wheels_Motors[2].setVelocity(self.Vy)
            self.wheels_Motors[3].setVelocity(self.Vx)
            if self.statusChanged:
                print(f"[INFO] fl:{self.Vx} ; bl:{self.Vy} ; fr :{self.Vy} ; br : {self.Vx}")
    
    def _get_Sensors_Values(self , idx):
        '''
            Func:
                Get the value of the corresponding sensor according to the corresponding index 
            Args:
                idx : required , The index value of the target sensor
            Return:
                the value of the corresponding sensor
        '''
        # for idx in range(len(self.wheels_Motors)):
        #     self.wheels_SensorsValues.append(self.wheels_Sensors[idx].getValue())
        # print("[INFO] WheelSensorsValues : " , self.wheels_SensorsValues)
        
        idx_sensor_values = self.wheels_Sensors[idx].getValue()
        
        return idx_sensor_values 
    
    def _explore_maze_algorithm(self):
        '''
            Func : 
                Get any available information to complete the maze exploration
            Args : 
                any value you want to use for maze exploration,such as sensor values, camera images, etc.
            Return : 
                ASCII/Other
        '''
        return None
    
    
def main():
    # <------ Instructions Introduction ------>
    # create the Robot instance.
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # <------ Init Robot Controller and show Instructions Introduction ------>
    robot_controller = RobotController(robot , max_speed=MAX_SPEED , sensor_threshold=SENSOR_THRESHOLD)
    robot_controller._instructions_intro()
    
    # <------ Main loop ------>
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        command = robot_controller._explore_maze_algorithm()
        robot_controller._keyboard_catcher(command)
        robot_controller._set_Mecanum_Velocoty()
        sensor_values = robot_controller._get_Sensors_Values(idx = 0)
        print(f"[INFO] sensor_values : {sensor_values}")
        
            
if __name__ == '__main__':
    main()

