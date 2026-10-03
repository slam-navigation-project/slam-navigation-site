import tkinter as tk


# -----------------------------------------------------
# 맵 시각화
# -----------------------------------------------------
def path_v(map, path, start, goal, W_map, H_map):
    CELL_SIZE = 25

    # -------------------------------
    # 윈도우 생성
    # -------------------------------
    root = tk.Tk()
    root.title("A* Algorithm test")

    canvas = tk.Canvas(
        root,
        width=W_map * CELL_SIZE,
        height=H_map * CELL_SIZE
    )

    canvas.pack()

    # -------------------------------
    # 맵 그리기
    # -------------------------------
    for y in range(H_map):
        for x in range(W_map):

            value = map[y][x]

            # 이동 가능 공간
            if value == 0:
                color = "white"

            # 장애물
            elif value == 100:
                color = "black"

            # 미탐색 공간
            elif value == -1:
                color = "gray"

            else:
                color = "yellow"

            canvas.create_rectangle(
                x * CELL_SIZE,
                y * CELL_SIZE,
                (x + 1) * CELL_SIZE,
                (y + 1) * CELL_SIZE,

                fill=color,
                outline="lightgray"
            )

    # -------------------------------
    # A* 경로 표시
    # -------------------------------
    for i in range(len(path) - 1):

        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        canvas.create_line(
            x1 * CELL_SIZE + CELL_SIZE / 2,
            y1 * CELL_SIZE + CELL_SIZE / 2,

            x2 * CELL_SIZE + CELL_SIZE / 2,
            y2 * CELL_SIZE + CELL_SIZE / 2,

            fill="blue",
            width=4
        )

    # -------------------------------
    # 시작점 표시
    # -------------------------------
    start_x, start_y = start

    canvas.create_rectangle(
        start_x * CELL_SIZE,
        start_y * CELL_SIZE,
        (start_x + 1) * CELL_SIZE,
        (start_y + 1) * CELL_SIZE,

        fill="green",
        outline="black"
    )
    # -------------------------------
    # 목적지 표시
    # -------------------------------
    goal_x, goal_y = goal

    canvas.create_rectangle(
        goal_x * CELL_SIZE,
        goal_y * CELL_SIZE,
        (goal_x + 1) * CELL_SIZE,
        (goal_y + 1) * CELL_SIZE,

        fill="red",
        outline="black"
    )
    root.mainloop()