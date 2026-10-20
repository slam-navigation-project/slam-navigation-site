import numpy as np
import heapq


# -----------------------------------------------------
# 함수 정의
# -----------------------------------------------------

## 유클리드 거리를 리턴(직선 거리)
def get_Euclidean(x1, y1, x2, y2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

## Diagonal 거리를 리턴(8방향 이동 시 휴리스틱 코스트)
def get_Diagonal(x1, y1, x2, y2):
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return min(dx, dy) * np.sqrt(2) + abs(dx - dy)

## Manhattan 거리를 리턴(4방향 이동 시 휴리스틱 코스트)
def get_Manhattan(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)




# -----------------------------------------------------
# 필요한 리스트 및 벽수 생성
# -----------------------------------------------------
def astar(map, W_map, H_map, start, goal):
    ## G : 시작 지점 ~ 해당 노드까지의 총 이동 거리. 
    ## H : 해당 노드 ~ 도착 지점까지의 거리(휴리스틱). 계산에는 여러 방법이 있으나 여기서는 맨해튼 거리를 사용.
    ## 위치값 표현 : (x좌표, y좌표) 리스트에서 접근할 경우 [y좌표][x좌표] 로 접근

    ## 노드를 담을 리스트 선언(open : 아직 탐색하지 않은 노드, closed : 탐색이 끝난 곳, 또는 막힌 곳)
    ## open_list에는 노드의 비용 및 부모노드 기록
    open_list, closed_list = [[None for _ in range(W_map)] for _ in range(H_map)], [[0 for _ in range(W_map)] for _ in range(H_map)]

    ## 최소비용인 노드를 찾기 위한 heap
    open_heap = []
    ## 최단경로 저장
    PATH = []

    ## 이동 방향 정의
    direct = [(0, -1), (1, 0), (0, 1), (-1, 0)] # 북->동->남->서

    ## 왼쪽 위를 (0, 0)으로 가정. 목표 위치 임의 설정
    # start = (1, 0)
    # goal = (4, 22)

    ## 현재 위치 x, y를 시작 지점으로 초기화
    x, y = start
    start_h = get_Manhattan(goal[0], goal[1], x, y)
    open_list[y][x] = {
                        "G" : 0,
                        "H" : start_h,
                        "F" : start_h,
                        "parent" : (x, y)
                        }


    def find_path(x, y):
        for i in range(4):
            # 노드 탐색
            node_x, node_y = x+direct[i][0], y+direct[i][1]
            # 만약 탐색 위치가 현재 맵 범위 안에 있을 경우
            if -1<node_x<W_map and -1<node_y<H_map:
                # open_list나 closed_list에 포함되지 않은 노드일 경우
                if open_list[node_y][node_x]==None and closed_list[node_y][node_x]==0:
                    # 만약 빈 공간일 경우 다음과 같이 기록
                    if map[node_y][node_x] == 0:
                        G = open_list[y][x]["G"] + get_Euclidean(x, y, node_x, node_y)
                        H = get_Manhattan(goal[0], goal[1], node_x, node_y)
                        F = G+H
                        open_list[node_y][node_x] = {
                            "G" : G,
                            "H" : H,
                            "F" : F,
                            "parent" : (x, y)
                        }
                        heapq.heappush(open_heap, (F, node_x, node_y))
                    # 만약 막힌 부분일 경우 다음과 같이 기록
                    else:
                        closed_list[node_y][node_x] = 1
                # 만약 현재 노드가 이미 탐색한 적 있는 노드라면 G값을 비교. 
                # 현재 노드를 거쳐갔을 때의 비용이 더 저렴하다면 부모 노드를 교체.
                elif open_list[node_y][node_x]:
                    G = open_list[y][x]["G"] + get_Euclidean(x, y, node_x, node_y)
                    F = G + open_list[node_y][node_x]["H"]
                    if G < open_list[node_y][node_x]["G"]:
                        open_list[node_y][node_x]["G"] = G
                        open_list[node_y][node_x]["F"] = F
                        open_list[node_y][node_x]["parent"] = (x, y)
                        heapq.heappush(open_heap,(F, node_x, node_y))
            else:
                pass

        ## 최단 거리 노드 찾기
        while open_heap:
            min_F, min_x, min_y = heapq.heappop(open_heap)

            # 이미 closed된 노드면 무시
            if closed_list[min_y][min_x]:
                continue

            # 현재 저장된 최신 F와 다르면 오래된 heap 데이터
            if open_list[min_y][min_x]["F"] != min_F:
                continue

            break

        # print(min_x, min_y)
        return min_x, min_y

    while(not(x == goal[0] and y == goal[1])):
        x, y = find_path(x, y)

        
    PATH.append(goal)
    tmp_x, tmp_y = goal
    while(not(tmp_x == start[0] and tmp_y == start[1])):
        tmp_x, tmp_y = open_list[tmp_y][tmp_x]["parent"] 
        PATH.append((tmp_x, tmp_y))

    PATH.reverse()
    return PATH
    # print(PATH)


