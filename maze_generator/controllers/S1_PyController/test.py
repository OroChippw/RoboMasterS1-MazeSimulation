
def _collect_MazeInfo(file_path="maze.txt"):
    maze = []
    with open(file_path , "r") as file: 
        for line in file:
            row = [char for char in line.strip()]
            print(f"length : {len(row)} , " , row)
            maze.append(row)
    
    for row in maze:
        print(' '.join(row))
    
    return maze

def find_path(maze):
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
                directions.append('Down')
            else:
                directions.append('Up')

    merged_directions = [directions[0]]
    for direction in directions[1:]:
        if direction != merged_directions[-1]:
            merged_directions.append(direction)
    
    return path[::-1], directions , merged_directions # 返回反向的路径和方向信息，从起点到终点

# 用迷宫示例调用算法
maze_example = [
    ['#', ' ', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'],
    ['#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'],
    ['#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#'],
    ['#', ' ', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'],
    ['#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#'],
    ['#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'],
    ['#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#'],
    ['#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'],
    ['#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#'],
    ['#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'],
    ['#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#'],
    ['#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'],
    ['#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', '#', '#', '', '#'],
    ['#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'],
    ['#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', '#', '#', ' ', '#'],
    ['#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'],
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#']
]

maze = _collect_MazeInfo()
print(type(maze))

print(len(maze_example) , len(maze_example[0]))

# result , directions = find_path(maze_example)
result , directions , merged_directions = find_path(maze)

if result:
    print("Path found:")
    for step in result:
        print(step)
    print(directions)
    print(merged_directions)
    
else:
    print("No path found.")
