
# -----------------------------------------------------
# 맵 불러오기
# -----------------------------------------------------
def map_loader(map_file, W_map, H_map):
    ## map 파일(.txt) : .py 파일과 같은 경로에 위치. 맵 데이터가 쉼표 단위로 구분되어있음. (100 : 막혀있음, 0 : 비어있는 공간, -1 : 미탐색공간)
    ## 맵 데이터를 배열로 받아오는 작업
    with open(map_file, "r", encoding="utf-8") as f:
        map_data = list(map(int, f.read().split(",")))

    ## 맵의 사이즈
    # W_map = 19  
    # H_map = 24  

    grid_map = []
    ## 1차원 배열 -> 2차원 배열
    for i in range(0, W_map*H_map, W_map):
        grid_map.append(map_data[i:i+W_map])

    grid_map.reverse()
    return grid_map