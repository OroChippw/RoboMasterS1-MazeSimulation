"""
    File : S1_PyController.py
    Author : OroChippw
    Date : 2023.11.29
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot , Keyboard , Supervisor
from controller import Supervisor
from math import pi

MAX_SPEED = 20
SENSOR_THRESHOLD = 15 # sensitivity of sensors surrounding robot for obstacle detection

class RobotController():
    '''
        The RobotController of RoboMaster S1
        #       [A]
        #    0-------2
        #        |
        #[B]     |    [D]
        #        |
        #    1-------3
        #       [C]
        [0-fl-front left] [1-bl-back left] [2-fr-front right] [3-br-back right] 
        [A-top_sensor] [B-left_sensor] [C-bot_sensor] [D-right_sensor]     
    '''
    def __init__(self , robot , timestep=32 ,max_speed=15 , 
                 sensor_threshold=15) -> None:
        # <------ Properties ------>
        self.robot = robot
        self.robot_name = "RoboMasterS1"
        self.timestep = timestep
        self.status = None
        self.statusTemp = None
        self.statusChanged = False
        self.wheel_radius = 0.058
        self.wheels_MotorNames = ['wheel_fl_motor', 
            'wheel_bl_motor', 'wheel_fr_motor', 'wheel_br_motor']
        self.wheels_SensorNames = ['wheel_fl_sensor', 
            'wheel_bl_sensor', 'wheel_fr_sensor', 'wheel_br_sensor']
        self.body_SensorNames = ["top_sensor" , "left_sensor" , "bot_sensor" , "right_sensor"]
        assert len(self.wheels_MotorNames) == len(self.wheels_SensorNames) , \
            f"[ERROR] The number of motors and sensors of the transmission wheel should be same"
        
        self.supervisor = Supervisor()
        self.robot_node = None
        self.translation_field = None
        self.rotation_field = None
        
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
        
        self.wheels_Motors = []
        self.wheels_Sensors = []
        self.body_Sensors = []
        self.wheels_MaxSpeeds = []
        self.wheels_SensorsValues = []
        self.body_SensorValues = []
        self.yaw_motor = None
        self.pitch_motor = None
        
        # <------ Thresholds ------>
        self.max_speed = max_speed
        self.sensor_threshold = sensor_threshold
        
        # <------ Signal / Status ------>
        self.prev_signal = None
        self.origin_directions = None
        self.merged_directions = None
        self.direction_length = 0
        self.direction_idx = 0
        self.rotation_idx_temp = 0
        self.is_rotating = None
        self.is_walking = None
        
        # <------ Time-related properties ------>
        self.approx_horizontal_or_vertical_counter = 0
        
        self._init_component()
        
    def _instructions_intro(self):
        '''
            Func:
                Prints instructions for controlling the robot using the keyboard and mouse.
            Args/Return:
                None
        '''
        print("<------ [DEMO] RoboMaster S1 Maze Simulation ------>")
        print("[INFO] #       [A]")
        print("[INFO] #    0-------2")
        print("[INFO] #        |")
        print("[INFO] #[B]     |    [D]")
        print("[INFO] #        |")
        print("[INFO] #    1-------3")
        print("[INFO] #       [C]")        
        print("[INFO] [0-fl-front left] [1-bl-back left] [2-fr-front right] [3-br-back right] ")
        print("[INFO] [A-top_sensor] [B-left_sensor] [C-bot_sensor] [D-right_sensor] ")
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
            print(f"[INFO] Width : {self.maincarema.getWidth()} , Height : {self.maincarema.getHeight()}")
            
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
                self.rotation_field = self.robot_node.getField('rotation')
            else:
                print("[INFO] Robot node not found.")
            
            self.initial_robot_global_position = self._get_Position()
            self.initial_robot_global_rotation = self._get_Rotation()
            
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
            self.Vx = 3
            self.Vy = -3
            self.w = 8
            self.status = "Rotate Counterclockwise" # 逆时针旋转
        elif (self.current_key == ord('E')):
            self.Vx = -3
            self.Vy = 3 
            self.w = 8
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
        
        if self.status != self.statusTemp:
            self.statusChanged = True
        self.statusTemp = self.status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

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
    
    def _get_Rotation(self):
        try:
            if self.robot_node is not None:
                robot_rotation = self.rotation_field.getSFVec3f()
        except Exception as e:
            print("[ERROR] " , e)
        
        return robot_rotation
    
    def _collect_MazeInfo(self , file_path="maze.txt"):
        self.maze = []
        with open(file_path , "r") as file: 
            for line in file:
                row = [char for char in line.strip()]
                self.maze.append(row)
        
        for row in self.maze:
            print(' '.join(row))
            
        return self.maze
        
    def _distance_to_wall(self, robot_position, wall_node):
        wall_position = wall_node.getPosition()
        return ((robot_position[0] - wall_position[0]) ** 2 +
                (robot_position[1] - wall_position[1]) ** 2) ** 0.5
        
    def _has_left_obstacle(self , left_sensor_values , LRdistanceThreshold):
        return (left_sensor_values < LRdistanceThreshold)
    
    def _has_right_obstacle(self , right_sensor_values , LRdistanceThreshold):
        return (right_sensor_values < LRdistanceThreshold)
    
    def _is_approx_horizontal_or_vertical(self , angle, threshold=2):
        '''
            映射到0-360度范围，并判断是否水平或垂直
        '''
        angle_360 = (angle + 2 * pi) % (2 * pi) * 180 / pi  
        is_horizontal = (abs(angle_360 - 0) < threshold) or \
            (abs(angle_360 - 180) < threshold)
        is_vertical = (abs(angle_360 - 90) < threshold) or \
            (abs(angle_360 - 270) < threshold)
        
        return (is_horizontal or is_vertical)
    
    def _find_path_with_direction(self , maze=None):
        def dfs(x, y, path):
            if x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0]) or maze[x][y] == '#' or visited[x][y]:
                return False

            visited[x][y] = True
            path.append((x, y))

            if x == 1 and y == 1:  # 已经到达终点
                return True

            # 尝试向上、向下、向左、向右探索
            if dfs(x - 1, y, path) or dfs(x + 1, y, path) or dfs(x, y - 1, path) or dfs(x, y + 1, path):
                return True

            # 若四个方向都无法到达终点，则回溯
            path.pop()
            return False

        # 初始化访问数组
        visited = [[False] * len(maze[0]) for _ in range(len(maze))]

        # 从右下角开始深度优先搜索
        path = []
        start_x, start_y = len(maze) - 2, len(maze[0]) - 2
        dfs(start_x, start_y, path)

        # 从起点到终点的方向列表
        directions = []
        for i in range(1, len(path)):
            prev_x, prev_y = path[i - 1]
            current_x, current_y = path[i]

            if current_x == prev_x:
                if current_y > prev_y:
                    directions.append('Right')
                else:
                    directions.append('Left')
            elif current_y == prev_y:
                if current_x > prev_x:
                    directions.append('Backward')
                else:
                    directions.append('Forward')

        merged_directions = [directions[0]]
        for direction in directions[1:]:
            if direction != merged_directions[-1]:
                merged_directions.append(direction)
        
        self.origin_directions = merged_directions
        self.merged_directions = merged_directions
        self.direction_length = len(self.origin_directions)
        self.prev_signal = merged_directions[0]
        
        # 返回反向的路径和方向信息，从起点到终点
        return path[::-1], directions , merged_directions 
     
    def _get_direction(self ,  path):
        directions = []
        for i in range(1, len(path)):
            curr_x, curr_y = path[i - 1]
            next_x, next_y = path[i]
            if curr_x < next_x:
                directions.append("Backward")
            elif curr_x > next_x:
                directions.append("Forward")
            elif curr_y < next_y:
                directions.append("Right")
            elif curr_y > next_y:
                directions.append("Left")
        return directions

    def _classify_rotation_direction(self , rotation_angle):
        """
            将机器人当前的旋转角度分为上下左右四个方向
            x的负半轴为0度
        """
        angle_360 = (rotation_angle + 2 * pi) % (2 * pi) * 180 / pi  # 映射到0-360度范围
        if 45 <= angle_360 < 135:
            return 0
        elif 135 <= angle_360 < 225:
            return 3
        elif 225 <= angle_360 < 315:
            return 2
        else:
            return 1
    
    def _delay(self , delay_ms):
        initTime = self.robot.getTime()
        while self.robot.step(self.timestep) != -1:
            if (self.robot.getTime() - initTime) * 1000.0 > delay_ms:
                break
    
    def _update_merged_directions(self, current_rotation):
        # 根据机器人当前朝向更新 merged_directions
        # 假设机器人车头朝上对应的角度是 0 或 12π rad,指令说朝上，但相对小车行径方向是右
        # 0-上，1左，2下，3右
        angle_mapping = {
            0: {'Forward': 'Forward', 'Backward': 'Backward', 'Left': 'Left', 'Right': 'Right'},
            1: {'Forward': 'Right', 'Backward': 'Left', 'Left': 'Forward', 'Right': 'Backward'},
            2: {'Forward': 'Backward', 'Backward': 'Forward', 'Left': 'Right', 'Right': 'Left'},
            3: {'Forward': 'Left', 'Backward': 'Right', 'Left': 'Backward', 'Right': 'Forward'},
        }

        updated_directions = [angle_mapping[current_rotation][dir] for dir in self.origin_directions]
        return updated_directions
    
    def _explore_maze_algorithm(self , FrontdistanceThreshold=2.5 , 
                LRdistanceThreshold=2.5):
        '''
            Func : 
                Get any available information to complete the maze exploration
            Args : 
                any value you want to use for maze exploration,such as sensor values, camera images, etc.
            Return : 
                ASCII/Other
        '''

        body_sensor_values = self._get_Sensors_Values(idx = 0 , compont="body")
        left_sensor_values = self._get_Sensors_Values(idx = 2 , compont="body")
        right_sensor_values = self._get_Sensors_Values(idx = 3 , compont="body")
        
        # 提供判断左右是否有障碍物的函数
        left_obstacle = self._has_left_obstacle(left_sensor_values , LRdistanceThreshold)
        right_obstacle = self._has_right_obstacle(right_sensor_values , LRdistanceThreshold)
        
        robot_current_rotation = self._get_Rotation()
       
        direct = self.merged_directions[self.direction_idx]
        
        if (direct == 'Left' or direct == "Right"):
            if (self.prev_signal == ord('Q') or self.prev_signal == ord('E')): 
                if (not self._is_approx_horizontal_or_vertical(robot_current_rotation[3])):
                    # if (self.prev_signal == ord('E')) : print(-1 , end=" ")
                    # else : print(-2 , end=" ")
                    self.is_rotating = True
                    self.is_walking = False
                    return self.prev_signal
                else :
                    # print("\n[INFO] Is Approx Horizontal or Vertical =》Set None")
                    self.approx_horizontal_or_vertical_executed = True
                    signal = None
                    self.prev_signal = signal 
                    self.is_walking = False
                    self.is_rotating = False
                    self.approx_horizontal_or_vertical_counter += 1
                    rotation_idx = self._classify_rotation_direction(robot_current_rotation[3])
                    if rotation_idx  != self.rotation_idx_temp:
                        self.rotation_idx_temp = rotation_idx
                        self.merged_directions = self._update_merged_directions(rotation_idx)
                        print(f"[ORIGIN] origin_directions : {self.origin_directions}")
                        print(f"[CHANGE] merged_directions : {self.merged_directions}")
                    if self.approx_horizontal_or_vertical_counter > 0:
                        self.approx_horizontal_or_vertical_counter = 0
                        return signal
            elif (direct == 'Right') : # 机器人右转
                signal = ord('E')
                self.prev_signal = signal
                self.is_rotating = True
                self.is_walking = None
                # print(1 , end=" ")
            elif (direct == 'Left') : # 机器人左转
                signal = ord('Q')
                self.prev_signal = signal
                self.is_rotating = True
                self.is_walking = None
                # print(2 , end=" ")
        elif (direct == 'Forward') :
            angle_360 = (robot_current_rotation[3] + 2 * pi) % (2 * pi) * 180 / pi
            # print(f"Enter Forward - rotation_idx : {angle_360}")
            
            if ((body_sensor_values > FrontdistanceThreshold)):
                signal = ord('W')    
                self.prev_signal = signal
                self.is_rotating = False
                self.is_walking = True
                # print(0 , end=" ")
            else:
                # print("\n[INFO] FrontdistanceThreshold =》Set None")
                signal = None
                self.prev_signal = signal 
                self.is_walking = False
                self.is_rotating = False
        elif (direct == 'Backward') :
            rotation_idx = 2
            self.merged_directions = self._update_merged_directions(rotation_idx)
            print(f"[ORIGIN] origin_directions : {self.origin_directions}")
            print(f"[CHANGE] merged_directions : {self.merged_directions}")
            signal = ord('W')
            self.prev_signal = signal
            self.is_rotating = None
            self.is_walking = True
            # print(3 , end=" ")
        
        if (not self.is_walking) and (not self.is_rotating) and (self.prev_signal is None):
            if (self.direction_idx < self.direction_length - 1):
                self.direction_idx += 1
                # print("Do direction_idx add")
                # print(f"\n[Triplet = CHANGE] direction_idx : {self.direction_idx} , direction : {self.merged_directions[self.direction_idx]}")
        
        return signal
                
                        
def main():
    # <------ Instructions Introduction ------>
    # create the Robot instance.
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
        
    # <------ Init Robot Controller and show Instructions Introduction ------>
    robot_controller = RobotController(
        robot , max_speed=MAX_SPEED , sensor_threshold=SENSOR_THRESHOLD)
    robot_controller._instructions_intro()
    
    # <------ Init Maze Information ------>
    maze = robot_controller._collect_MazeInfo()
    
    result , direction , merged_directions = robot_controller._find_path_with_direction(maze)

    if result:
        print(f"[INFO] Path found , Mergeed Direction : {merged_directions}")
    else:
        print("[ERROR] No Path found")
        exit(0)
    
    # <------ Main loop ------>
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        command = robot_controller._explore_maze_algorithm()
        robot_controller._keyboard_catcher(command)
        robot_controller._set_Mecanum_Velocoty()
        
            
if __name__ == '__main__':
    main()

