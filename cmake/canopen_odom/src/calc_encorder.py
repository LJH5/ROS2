import math

# 바퀴 사이 거리 (m)
WHEEL_BASE = 0.4

# 바퀴 지름 (m)
WHEEL_DIAMETER = 0.173

# 1바퀴 엔코더
ENCORDER_PER_CYCLE = 16384

x, y, theta = 0, 0, 0

def get_odometry(left_distance, right_distance):
    """
    (x, y, theta): 현재 위치 (x, y)와 방향 (theta)
    """
    # 회전 각도 계산
    d_theta = (right_distance - left_distance) / WHEEL_BASE

    # 이동 거리 계산
    d_x = ((left_distance + right_distance) / 2) * math.cos(d_theta)
    d_y = ((left_distance + right_distance) / 2) * math.sin(d_theta)

    # 현재 위치 계산
    x = 0  # 이전 위치 x 값을 입력
    y = 0  # 이전 위치 y 값을 입력
    theta = 0  # 이전 방향 값을 입력

    return x + d_x, y + d_y, theta + d_theta


def calc_odom(left_encorder, right_encorder):
    global x, y, theta
    left_distance = left_encorder * WHEEL_DIAMETER / ENCORDER_PER_CYCLE
    right_distance = right_encorder * WHEEL_DIAMETER / ENCORDER_PER_CYCLE

    x, y, theta = get_odometry(left_distance, right_distance)

    print(f"현재 위치: ({x}, {y})")
    print(f"현재 방향: {theta}")

    return x, y, theta
