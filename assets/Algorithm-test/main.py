from astar import astar
from path_visualization import path_v
from map_loader import map_loader

# 맵의 사이즈
W_map = 19  
H_map = 24  

# 왼쪽 위를 (0, 0)으로 가정. 목표 위치 임의 설정
start = (1, 0)
goal = (4, 22)

grid_map = map_loader("map.txt", W_map, H_map)

path = astar(grid_map, W_map, H_map, start, goal)
print(path)
path_v(grid_map, path, start, goal, W_map, H_map)