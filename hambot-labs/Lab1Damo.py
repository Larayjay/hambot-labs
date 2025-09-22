import numpy as np
import time
from robot_systems.robot import HamBot

# HamBot 초기화 (센서 비활성화)
bot = HamBot(lidar_enabled=False, camera_enabled=False)

# 속도 설정
MAX_V = 0.05  # m/s, 필요시 조절
MAX_W = 0.3   # rad/s, 필요시 조절

# ---- 이동 함수 ----
def move_straight(D, speed=MAX_V):
    """
    단순 시간 기반 직선 이동
    """
    bot.set_left_motor_velocity(speed)
    bot.set_right_motor_velocity(speed)
    
    t_total = D / speed
    start_time = time.time()
    while time.time() - start_time < t_total:
        time.sleep(0.01)
    bot.stop()

def rotate(angle, speed=MAX_W):
    """
    단순 시간 기반 회전
    angle > 0: CCW, angle < 0: CW
    """
    w = speed if angle > 0 else -speed
    bot.set_left_motor_velocity(-w)
    bot.set_right_motor_velocity(w)
    
    t_total = abs(angle) / speed
    start_time = time.time()
    while time.time() - start_time < t_total:
        time.sleep(0.01)
    bot.stop()

def move_arc(R, theta, direction="CCW", max_v=MAX_V):
    """
    단순화: 회전 반지름 기반 아크 이동
    """
    if direction.upper() == "CCW":
        w = MAX_W
    else:
        w = -MAX_W
    v = w * R
    bot.set_left_motor_velocity(v - w * bot.axel_length / 2)
    bot.set_right_motor_velocity(v + w * bot.axel_length / 2)
    
    t_total = abs(theta) / abs(w)
    start_time = time.time()
    while time.time() - start_time < t_total:
        time.sleep(0.01)
    bot.stop()

# ---- Path ----
P = [(2.0, -2.0, np.pi),
     (-1.5, -2.0, np.pi),
     (-2.0, -1.5, np.pi/2),
     (-2.0, -0.5, np.pi/2),
     (-1.0, -0.5, 3*np.pi/2),
     (-0.5, -1.0, 7*np.pi/4)]

# ---- 자동 주행 루프 ----
for i in range(len(P)-1):
    start = np.array(P[i][:2])
    end = np.array(P[i+1][:2])
    
    vec = end - start
    distance = np.linalg.norm(vec)
    
    # 현재 방향과 목표 방향 계산
    angle_target = np.arctan2(vec[1], vec[0])
    angle_current = P[i][2]
    delta_angle = (angle_target - angle_current + np.pi) % (2*np.pi) - np.pi
    
    rotate(delta_angle)         # 방향 맞추기
    move_straight(distance)     # 직선 이동

print("Path 완료!")
bot.stop()

