"""
    File : Maze_Generator.py
    Author : OroChippw
    Date : 2023.11.29
"""
import random
import os.path as osp


class MazeGenerator():
    def __init__(self , m=4 , n=4 , title_size=10 , 
                 save_path="output" , save_name="maze.wbt") -> None:
        self.maze = None
        self.m = m
        self.n = n        
        self.M = 2 * m + 1
        self.N = 2 * n + 1
        
        self.title_size = title_size
        self.unit_size = title_size / 20
        
        self.save_path = save_path
        self.save_name = save_name
        
        self._init_maze(self.M , self.N)
    
    def _init_maze(self , M , N):
        self.maze = [
            ['#' if not (i & 1) or not (j & 1) else ' ' for j in range(N)] for i in range(M)
        ]
        for i in range(1 , M , 2):
            for j in range(1 , N , 2):
                self.maze[i][j] = ' '
    
    def _get_idx(self , x , y , cell_list):
        for idx , cell in enumerate(cell_list):
            if cell[1][0] == x and cell[1][1] == y:
                return idx
        # print(f"[ERROR] Couldn't find the index ({x} , {y})")
        
        return -1
    
    def _get_filename(self):
        return self.filename
    
    def _create_maze(self):
        '''
            Func:
            Args:
            Return:
                None
        '''
        n_visited = 0
        m = self.n
        n = self.n
        
        start_x = self.M - 2
        start_y = self.N - 2
        end_x = 1
        end_y = 1
        
        cell_list = [
            (k, (i, j)) for k, (i, j) in enumerate(
                [(i, j) for i in range(1, self.M, 2) for j in range(1, self.N, 2)])
        ]
        stack = []
        visited = [False] * (m * n)
        
        
        top_left = (((m // 2) - 1) * n) + (n // 2) - 1
        top_right = top_left + 1
        down_left = top_left + n
        down_right = down_left + 1
        
        self.maze[cell_list[top_left][1][0] + 1][cell_list[top_left][1][1] + 0] = ' '
        self.maze[cell_list[top_left][1][0] + 0][cell_list[top_left][1][1] + 1] = ' '
        visited[cell_list[top_left][0]] = True
        n_visited += 1

        self.maze[cell_list[top_right][1][0] + 1][cell_list[top_right][1][1] + 0] = ' '
        visited[cell_list[top_right][0]] = True
        n_visited += 1

        self.maze[cell_list[down_left][1][0] + 0][cell_list[down_left][1][1] + 1] = ' '
        visited[cell_list[down_left][0]] = True
        n_visited += 1

        visited[cell_list[down_right][0]] = True
        n_visited += 1

        open_from = random.randint(0, 7)
        rand_idx = -1
        
        if open_from == 0:
            rand_idx = top_left - n
            self.maze[cell_list[rand_idx][1][0] + 1][cell_list[rand_idx][1][1] + 0] = ' '
        elif open_from == 1:
            rand_idx = top_right - n
            self.maze[cell_list[rand_idx][1][0] + 1][cell_list[rand_idx][1][1] + 0] = ' '
        elif open_from == 2:
            rand_idx = top_right + 1
            self.maze[cell_list[rand_idx][1][0] + 0][cell_list[rand_idx][1][1] + 1] = ' '
        elif open_from == 3:
            rand_idx = down_right + 1
            self.maze[cell_list[rand_idx][1][0] + 0][cell_list[rand_idx][1][1] - 1] = ' '
        elif open_from == 4:
            rand_idx = down_right + n
            self.maze[cell_list[rand_idx][1][0] - 1][cell_list[rand_idx][1][1] + 0] = ' '
        elif open_from == 5:
            rand_idx = down_left + n
            self.maze[cell_list[rand_idx][1][0] - 1][cell_list[rand_idx][1][1] + 0] = ' '
        elif open_from == 6:
            rand_idx = down_left - 1
            self.maze[cell_list[rand_idx][1][0] + 0][cell_list[rand_idx][1][1] + 1] = ' '
        elif open_from == 7:
            rand_idx = top_left - 1
            self.maze[cell_list[rand_idx][1][0] + 0][cell_list[rand_idx][1][1] + 1] = ' '
        
        stack.append(cell_list[rand_idx])
        visited[rand_idx] = True
        n_visited += 1
        
        while n_visited < (self.m * self.n):
            neighbours = []
            
            # North
            if stack[-1][1][0] > 1:
                if self.maze[stack[-1][1][0] - 2][stack[-1][1][1] + 0] and not visited[self._get_idx(stack[-1][1][0] - 2, stack[-1][1][1] + 0, cell_list)]:
                    neighbours.append(0)
            # East
            if stack[-1][1][1] < self.N - 2:
                if self.maze[stack[-1][1][0] + 0][stack[-1][1][1] + 2] and not visited[self._get_idx(stack[-1][1][0] + 0, stack[-1][1][1] + 2, cell_list)]:
                    neighbours.append(1)
            # South
            if stack[-1][1][0] < self.M - 2:
                if self.maze[stack[-1][1][0] + 2][stack[-1][1][1] + 0] and not visited[self._get_idx(stack[-1][1][0] + 2, stack[-1][1][1] + 0, cell_list)]:
                    neighbours.append(2)
            # West
            if stack[-1][1][1] > 1:
                if self.maze[stack[-1][1][0] + 0][stack[-1][1][1] - 2] and not visited[self._get_idx(stack[-1][1][0] + 0, stack[-1][1][1] - 2, cell_list)]:
                    neighbours.append(3)

            # Remove neighbours that are already in the stack
            # neighbours = [dir for dir in neighbours if cell_list[self._get_idx(stack[-1][1][0] + (dir == 2) - (dir == 0), stack[-1][1][1] + (dir == 1) - (dir == 3), cell_list)][0] not in stack]
            
            if neighbours:
                next_cell_dir = random.choice(neighbours)

                if next_cell_dir == 0:  # North
                    self.maze[stack[-1][1][0] - 1][stack[-1][1][1] + 0] = ' '
                    stack.append(cell_list[self._get_idx(stack[-1][1][0] - 2, stack[-1][1][1] + 0, cell_list)])
                elif next_cell_dir == 1:  # East
                    self.maze[stack[-1][1][0] + 0][stack[-1][1][1] + 1] = ' '
                    stack.append(cell_list[self._get_idx(stack[-1][1][0] + 0, stack[-1][1][1] + 2, cell_list)])
                elif next_cell_dir == 2:  # South
                    self.maze[stack[-1][1][0] + 1][stack[-1][1][1] + 0] = ' '
                    stack.append(cell_list[self._get_idx(stack[-1][1][0] + 2, stack[-1][1][1] + 0, cell_list)])
                elif next_cell_dir == 3:  # West
                    self.maze[stack[-1][1][0] + 0][stack[-1][1][1] - 1] = ' '
                    stack.append(cell_list[self._get_idx(stack[-1][1][0] + 0, stack[-1][1][1] - 2, cell_list)])
                
                
                visited[stack[-1][0]] = True
                n_visited += 1
            else:
                stack.pop()
            
        self.maze[start_x + 1][start_y] = ' '
        self.maze[end_x - 1][end_y] = ' '

        return 
        
    def _display_maze(self):
        '''
            Func:
            Args:
            Return:
                None
        '''
        self.filename = osp.join(self.save_path , self.save_name)
        with open(self.filename  , "w") as out:
            out.write(
                "#VRML_SIM R2023b utf8 \n\n"
                "EXTERNPROTO \"../protos/RectangleArena.proto\" \n"
                "EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto\" \n"
                "EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto\" \n"
                "EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/apartment_structure/protos/Wall.proto\" \n\n"
                " WorldInfo { \n"
                "    info [ \n"
                "        \"time:2023-11-27\" \n"
                "        \"author:OroChippw\" \n"
                "    ] \n"
                "    coordinateSystem \"ENU\" \n"
                "    contactProperties [ \n"
                "        ContactProperties { \n"
                "        material1 \"InteriorWheelMat\" \n"
                "        coulombFriction [ \n"
                "            0, 2, 0 \n"
                "        ] \n"
                "        frictionRotation -0.785 0 \n"
                "        bounce 0 \n"
                "        forceDependentSlip [ \n"
                "            10, 0 \n"
                "        ] \n"
                "        } \n"
                "        ContactProperties { \n"
                "        material1 \"ExteriorWheelMat\" \n"
                "        coulombFriction [ \n"
                "            0, 2, 0 \n"
                "        ] \n"
                "        frictionRotation 0.785 0 \n"
                "        bounce 0 \n"
                "        forceDependentSlip [ \n"
                "            10, 0 \n"
                "        ] \n"
                "        } \n"
                "    ] \n"
                "    } \n"
                "    Viewpoint { \n"
                "    orientation -0.2533698099047507 -0.13341086831684887 0.9581259205573007 4.076081923379274 \n"
                "    position -3.110160230052604 -1.5631316631937642 1.13895648594493 \n"
                "   } \n"
                "    RectangleArena { \n"
                "    translation -0.1 0.16 0 \n"
                "    floorSize 13 13 \n"
                "    floorTileSize 1.5 1.5 \n"
                "    } \n"
                "    TexturedBackgroundLight { \n"
                "    } \n"
                "    TexturedBackground { \n"
                "    } \n"
            )
            
            counter = 0
            # Horizontal
            for i in range(self.M):
                for j in range(self.N - 1):
                    if (self.maze[i][j] == '#' and self.maze[i][j + 1] == '#'):
                        size_x = 0.5 if self.maze[i][j] == '#' and self.maze[i][j + 1] == '#' else 0.01
                        out.write(
                            f"Wall {{  \n translation {(-j - 0.5 + self.N / 2) * self.unit_size}  {(i - self.M / 2) * self.unit_size} 0 \n rotation 0 1 0 0 \n size {size_x} 0.1 0.4 \n name \" wall({counter})\" \n}}\n"
                        )
                        counter += 1
            
            # Vertical
            for i in range(self.M - 1):
                for j in range(self.N):
                    if (self.maze[i][j] == '#' and self.maze[i + 1][j] == '#'):
                        size_y = 0.5 if self.maze[i][j] == '#' and self.maze[i + 1][j] == '#' else 0.01
                        out.write(
                            f"Wall {{\n  translation {(-j + self.N / 2) * self.unit_size} {(i + 0.5 - self.M / 2) * self.unit_size} 0 \n  rotation 0 1 0 0 \n  size 0.1 {size_y} 0.4 \n name \" wall({counter})\" \n  }}\n"
                        )
                        counter += 1
            
        for i in range(self.M):
            for j in range(self.N):
                print(self.maze[i][j], end=' ')
            print()
    
    # def _generate_robot(self , robot_filepath):
    #     """
    #         Func :    
    #         Args :   
    #         Return :   
    #     """
    #     assert robot_filepath.lower().endswith(".txt"), \
    #         f"[ERROR] "
    #     with open(robot_filepath , "r") as robot_file , open(self.filename , 'w') as out:
    #         for line in robot_file:
    #             modified_line = line.replace('"', r'\"')
    #             modified_line = '"' + modified_line.strip() + '\\n"\n'
    #             out.write(modified_line)
                
    #     print("[INFO] Here's the robot you asked for. A maze.wbt has been created. Enjoy! :D")
        
    #     return 
                
def main():
    # <------ Parameter Settings ------>
    # GENERATE_ROBOT = False
    
    print("[INFO] Maze Generator")
    m = int(input("[INPUT] Enter the number of rows (greater than 4): "))
    n = int(input("[INPUT] Enter the number of columns (greater than 4): "))
    
    while m < 4 or n < 4:
        print("[ERROR] Desired dimensions are not possible. Please enter again.")
        m = int(input("Enter the number of rows (greater than 4): "))
        n = int(input("Enter the number of columns (greater than 4): "))
        
    print(f"[INFO] Generate a {m} x {n} maze")
    
    # <------ Maze Generate ------>
    maze_generator = MazeGenerator(m , n)
    
    maze_generator._create_maze()
    print("[INFO] Here's the maze you asked for. A maze.wbt has been created. Enjoy! :D")
    maze_generator._display_maze()  
    
    # if GENERATE_ROBOT:
    #     robot_filepath = "RoboMasterS1.txt"
    #     maze_generator._generate_robot(robot_filepath)
    
if __name__ == '__main__':
    main()