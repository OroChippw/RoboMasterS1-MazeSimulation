"""
    File : S1_PyController.py
    Author : OroChippw
    Date : 2023.11.29
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot , Keyboard , Supervisor
from controller import Supervisor
from controller import Camera , CameraRecognitionObject
from math import sin , cos , atan2 , pi

MAX_SPEED = 20
SENSOR_THRESHOLD = 15 # sensitivity of sensors surrounding robot for obstacle detection

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
                 sensor_threshold=15) -> None:
        # <------ Properties ------>
        self.robot = robot
        self.robot_name = "RoboMasterS1"
        self.timestep = timestep
        self.status = None
        self.temp = None
        self.statusChanged = False
        self.mode = mode
        self.wheel_radius = 0.058
        self.wheels_MotorNames = ['wheel_fl_motor', 
            'wheel_bl_motor', 'wheel_fr_motor', 'wheel_br_motor']
        self.wheels_SensorNames = ['wheel_fl_sensor', 
            'wheel_bl_sensor', 'wheel_fr_sensor', 'wheel_br_sensor']
        self.body_SensorNames = ["top_sensor"]
        assert len(self.wheels_MotorNames) == len(self.wheels_SensorNames) , \
            f"[ERROR] The number of motors and sensors of the transmission wheel should be same"
        
        self.supervisor = Supervisor()
        self.robot_node = None
        self.translation_field = None
        
        # linear velocities (Vx and Vy) , Positive are ahead and right
        self.Vx = 0 
        self.Vy = 0
        # angular velocity (w) , Positive are counterclockwise
        self.w = 0
        self.current_key = None # Capture keyboard signals
        self.current_pitch_angle = None
        
        # <------ Device 、Motors and Sensors ------>
        self.keyboard = None
        self.maincarema = None
        self.recognition = None
        
        self.wheels_Motors = []
        self.wheels_Sensors = []
        self.body_Sensors = []
        self.wheels_MaxSpeeds = []
        self.wheels_SensorsValues = []
        self.top_SensorValue = []
        self.yaw_motor = None
        self.pitch_motor = None
        
        # <------ Thresholds ------>
        self.max_speed = max_speed
        self.sensor_threshold = sensor_threshold
        
        # <------ Signal ------>
        self.prev_signal = None  # 用于存储上一个返回的信号
        self.distance_stack = []  # 用于存储N步操作距离的栈结构
        
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
            
            self.robot_name = self.robot.getName()
            print(f"[INFO] Robot Name : {self.robot_name}")
            
            # Gets the robot's keyboard device and robot's camera device
            self.keyboard = self.robot.getKeyboard()
            self.keyboard.enable(1)
            
            self.maincarema = self.robot.getDevice(maincarema_name)
            self.maincarema.enable(self.timestep)
            self.maincarema.recognitionEnable(self.timestep)
            # if self.maincarema.hasRecognition():
            #     self.recognition = self.maincarema.getRecognition()
            print(f"[INFO] Carema hasRecognition : {self.maincarema.hasRecognition()}")
            
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
            
            for idx in range(len(self.body_SensorNames)):
                self.body_Sensors.append(self.robot.getDevice(self.body_SensorNames[idx]))
                self.body_Sensors[idx].enable(self.timestep)
             
            print(f"[INFO] Complete initialization of sensors and motors")
            print(f"[INFO] WheelsMaxSpeeds : {self.wheels_MaxSpeeds}")
            
            self.robot_node = self.supervisor.getFromDef(self.robot_name)
            if self.robot_node is not None:
                self.translation_field = self.robot_node.getField('translation')        
            else:
                print("[INFO] Robot node not found.")
            
            self.initial_robot_global_position = self._get_Position()
            
            
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
            self.w = 0
            self.status = "Forward"
        elif (self.current_key == ord('S')):
            self.Vx = self.Vy = -5
            self.w = 0
            self.status = "Backward"
        elif (self.current_key == ord('D')):
            self.Vx = -5
            self.Vy = 5
            self.w = 0
            self.status = "Pan Right"
        elif (self.current_key == ord('A')):
            self.Vx = 5
            self.Vy = -5
            self.w = 0
            self.status = "Pan Left"
        elif (self.current_key == ord("Q")):
            self.Vx = 5
            self.Vy = -5
            self.w = 15
            self.status = "Rotate Counterclockwise" # 逆时针旋转
        elif (self.current_key == ord('E')):
            self.Vx = -5
            self.Vy = 5 
            self.w = 15
            self.status = "Rotate Clockwise" # 顺时针旋转
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
        elif (self.status == "Pan Left" or self.status == "Pan Right"):
            self.wheels_Motors[0].setVelocity(self.Vx)
            self.wheels_Motors[1].setVelocity(self.Vy)
            self.wheels_Motors[2].setVelocity(self.Vy)
            self.wheels_Motors[3].setVelocity(self.Vx)
        elif (self.status == "Rotate Counterclockwise" or self.status == "Rotate Clockwise"):
            w = self.w
            if (self.status == "Rotate Counterclockwise"):
                wheel_velocities = [
                    w * self.wheel_radius , w * self.wheel_radius, # 左前轮 , 左后轮
                    -w * self.wheel_radius , -w * self.wheel_radius # 右前轮,右后轮
                ]
            else :
                wheel_velocities = [
                    -w * self.wheel_radius , -w * self.wheel_radius,
                    w * self.wheel_radius , w * self.wheel_radius
                ]
            # Set individual wheel velocities
            for i in range(len(wheel_velocities)):
                self.wheels_Motors[i].setVelocity(min(wheel_velocities[i] , self.wheels_MaxSpeeds[i]))
            # if self.statusChanged:
            #     print(f"[INFO] fl:{wheel_velocities[0]} ; bl:{wheel_velocities[1]} ; \
            #         fr :{wheel_velocities[2]} ; br : {wheel_velocities[3]} ; w : {self.w}")
    
    def _get_Sensors_Values(self , idx = None , compont = None):
        '''
            Func:
                Get the value of the corresponding sensor according to the corresponding index 
            Args:
                idx : required , The index value of the target sensor
            Return:
                the value of the corresponding sensor
        '''
        if (compont == "wheels"):
            idx_sensor_values = self.wheels_Sensors[idx].getValue()
        elif (compont == "body"):
            idx_sensor_values = self.body_Sensors[idx].getValue()
        else :
            idx_sensor_values = 0
        
        return idx_sensor_values 
    
    def _get_Position(self):
        try:
            if self.robot_node is not None:
                robot_position = self.translation_field.getSFVec3f()
        except Exception as e:
            print("[ERROR] " , e)
        
        return robot_position
    
    def _get_Recognition(self):
        try:
            objects = self.maincarema.getRecognitionObjects()
            # print("[INFO] Recognition Object Num : " , self.maincarema.getRecognitionNumberOfObjects())
            # for obj in objects:
                # print("ID : " , obj.getId())
                # print("getColors : " , obj.getColors())
                
                # if obj.getId() == CameraRecognitionObject.Wall:
                #     print("[INFO] Solid object detected!")
        except Exception as e:
            print("[ERROR] ", e)
            
        return
    
    def _collect_MazeInfo(self , wall_nodes , maze_size , maze_center_point):
        self.wall_nodes = wall_nodes
        self.maze_size = maze_size
        self.maze_center_point = maze_center_point
        
    def _distance_to_wall(self, robot_position, wall_node):
        wall_position = wall_node.getPosition()
        return ((robot_position[0] - wall_position[0]) ** 2 +
                (robot_position[1] - wall_position[1]) ** 2) ** 0.5
    
    def _move_forward(self, distance=1.0):
        # 执行前进操作，距离为distance
        # ... （执行前进的代码）

        # 存储单步操作的信号和距离到栈中
        self.distance_stack.append((self.prev_signal, distance))
    
    def _move_backward(self):
        if self.distance_stack:
            # 弹出栈顶的信号和距离，用于后退操作
            prev_signal, distance = self.distance_stack.pop()
    
    def _move_left(self):
        # 执行左转操作
        # ... （执行左转的代码）

        # 存储单步操作的信号和距离到栈中
        self.distance_stack.append((self.prev_signal, 1.0))
    
    def _move_right(self):
        # 执行右转操作
        # ... （执行右转的代码）

        # 存储单步操作的信号和距离到栈中
        self.distance_stack.append((self.prev_signal, 1.0))
    
    def _explore_maze_algorithm(self , FrontdistanceThreshold=8.0 , 
                LRdistanceThreshold=3.0 , num_closest_walls=3):
        '''
            Func : 
                Get any available information to complete the maze exploration
            Args : 
                any value you want to use for maze exploration,such as sensor values, camera images, etc.
            Return : 
                ASCII/Other
        '''
        
        body_sensor_values = self._get_Sensors_Values(idx = 0 , compont="body")
        robot_current_position = self._get_Position()
        
        if (body_sensor_values > FrontdistanceThreshold):
            return ord('W')
        else :
            obstacles_info = []
            left_obstacle = False
            right_obstacle = False
            # 仅考虑距离机器人最近的若干个墙体
            closest_walls = sorted(self.wall_nodes, 
                key=lambda wall_node: self._distance_to_wall(robot_current_position, wall_node))[:num_closest_walls]
            
            for wall_node in closest_walls:
                wall_position = wall_node.getPosition()
                dx = wall_position[0] - robot_current_position[0]
                dy = wall_position[1] - robot_current_position[1]
                angle_to_wall = atan2(dy, dx) * (180 / pi)
                
                distance_to_wall = ((robot_current_position[0] - wall_position[0]) ** 2 +
                                (robot_current_position[1] - wall_position[1]) ** 2) ** 0.5

                wall_name = wall_node.getField('name').getSFString()
                
                obstacle_info = {'name': wall_name, 'distance': distance_to_wall, 'angle': angle_to_wall}
                obstacles_info.append(obstacle_info)
                
                
            # 判断左右方向是否也有障碍物
            
            # left_obstacle = any(0 <= info['angle'] <= 45 or 315 <= info['angle'] <= 360 for info in obstacles_info)
            # right_obstacle = any(135 <= info['angle'] <= 225 for info in obstacles_info)
            
            # 判断机器人朝向是否与 y 轴近似平行
            robot_angle_to_y_axis = atan2(robot_current_position[1], robot_current_position[0]) * (180 / pi)
            adjusted_angle_to_y_axis = (robot_angle_to_y_axis + 360) % 360  # 调整为[0, 360)范围
            left_obstacle = any(0 <= (info['angle'] + adjusted_angle_to_y_axis) % 360 <= 45 or 315 <= (info['angle'] + adjusted_angle_to_y_axis) % 360 <= 360 for info in obstacles_info)
            right_obstacle = any(135 <= (info['angle'] + adjusted_angle_to_y_axis) % 360 <= 225 for info in obstacles_info)
            # print(f"left_obstacle : {left_obstacle} , right_obstacle : {right_obstacle}")
            # print("abs(robot_angle_to_y_axis) : " , abs(robot_angle_to_y_axis))
            if left_obstacle and right_obstacle:
                if (self.prev_signal == ord('Q') or self.prev_signal == ord('E')) and (adjusted_angle_to_y_axis not in [0,90,180,270,360]):
                    if self.prev_signal is not None:
                        # 更新prev_signal信号的持续运动距离至栈顶元素
                        self.distance_stack[-1][1] += 1.0
                    return self.prev_signal
                else:
                    self.prev_signal = ord('A')
                    return ord('A')  # 机器人后退
            elif right_obstacle:
                self.prev_signal = ord('Q')
                return ord('Q')  # 机器人左转
            elif left_obstacle:
                self.prev_signal = ord('E')
                return ord('E')  # 机器人右转
                
        
def main():
    # <------ Instructions Introduction ------>
    # create the Robot instance.
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    robot.step(100)
    
    # <------ Init Maze Information ------>
    wall_nodes = []
    min_coords = [float('inf'), float('inf'), float('inf')]
    max_coords = [-float('inf'), -float('inf'), -float('inf')]
    
    maze_supervisor = Supervisor()
    maze_group = maze_supervisor.getFromDef('Maze01')
    maze_size = [float('inf'), float('inf'), float('inf')] # length、width、height
    
    if (maze_group is not None):
        print(f"[INFO] Maze_group getCount() : " , maze_group.getField('children').getCount())
        for idx in range(maze_group.getField('children').getCount()):
            node = maze_group.getField('children').getMFNode(idx)
            if (node.getTypeName().lower() == "wall"):
                wall_nodes.append(node)
        
        for wall_node in wall_nodes:
            # wall_name = wall_node.getField('name').getSFString()
            wall_position = wall_node.getPosition()
            # print(f"[INFO] WALL Name: {wall_name}, Position: {wall_position}")
            for i in range(3):
                min_coords[i] = min(min_coords[i], wall_position[i])
                max_coords[i] = max(max_coords[i], wall_position[i])

        center_point = [(min_coords[i] + max_coords[i]) / 2 for i in range(3)]
        maze_size[0] = max_coords[0] - min_coords[0]
        maze_size[1] = max_coords[1] - min_coords[1]
        maze_size[2] = (max_coords[2] + min_coords[2]) / 2

        print("[INFO] Maze Center Point: X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(*center_point))
        print("[INFO] Maze Length: {:.2f} , Width: {:.2f} , Height : ".format(maze_size[0] , maze_size[1] , maze_size[2]))
    else :
        print("[INFO] Maze Group is None")
    
   
    # <------ Init Robot Controller and show Instructions Introduction ------>
    robot_controller = RobotController(
        robot , max_speed=MAX_SPEED , sensor_threshold=SENSOR_THRESHOLD)
    robot_controller._instructions_intro()
    robot_controller._collect_MazeInfo(
        wall_nodes=wall_nodes , maze_size=maze_size , maze_center_point=center_point)
    # <------ Main loop ------>
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        command = robot_controller._explore_maze_algorithm()
        robot_controller._keyboard_catcher(command)
        robot_controller._set_Mecanum_Velocoty()
        
            
if __name__ == '__main__':
    main()

