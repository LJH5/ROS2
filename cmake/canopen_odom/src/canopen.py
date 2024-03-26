#!/usr/bin/env python3

import socket
import time
import odom
import  ctypes

# Client TCP connect
HOST = "192.168.100.120"
PORT = 4001
TIMEOUT = 3
TIMESLEEP = 0.5

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

# global val
ecan_type = "04"
ecan_dlc = "08"

last_left_encoder = 0
last_right_encoder = 0

# 1Byte = 8bit => 4bit / 4bit => 16진수 숫자 2개

### define function ######################################
def position_mode():
    print("###### Position Mode ######")

    cob_id = "601"

    # init
    init_ecan()

    # set profile postion
    send_data(cob_id, "6060", "00", "2f", "1")

    # set motor accel and decel time 0
    set_motor_accel_time(0)
    set_motor_decel_time(0)

    # Set the motor Maximum speed
    speed = "200"
    # left motor
    send_data(cob_id, "6081", "01", "23", speed)
    # right motor
    send_data(cob_id, "6081", "02", "23", speed)

    # enable motor
    enable_motor()

    # Set the motor Target position
    l_pos = input("left position: ")
    r_pos = input("right position: ")
    # left motor
    send_data(cob_id, "607a", "01", "23", l_pos)
    # right motor
    send_data(cob_id, "607a", "02", "23", r_pos)

    # Start relative movement
    send_data(cob_id, "6040", "00", "2b", "4f")
    send_data(cob_id, "6040", "00", "2b", "5f")


def velocity_mode():
    print("###### Velocity Mode ######")

    cob_id = "601"

    # init
    init_ecan()

    # choose async / sync
    set_async()

    # set velocity mode
    send_data(cob_id, "6060", "00", "2f", "3")

    # set motor accel and decel time 0
    set_motor_accel_time(0)
    set_motor_decel_time(0)

    enable_motor()

    # Set the left motor Target speed
    # left wheel reverse dir
    left_speed = int(input("left_speed: "))
    right_speed = int(input("right_speed: "))
    send_data(cob_id, "60ff", "01", "23", make_data_form(-left_speed))
    send_data(cob_id, "60ff", "02", "23", make_data_form(right_speed))

def torque_mode():
    pass

def custom_mode():
    obj_idx = input("obj_idx: ")
    sub_idx = input("sub_idx: ")
    command = input("command: ")
    if command == "40":
        data = "0"
    else:
        data = make_data_form(input("data: "))
    send_data("601", obj_idx, sub_idx, command, data)

def send_data(cob_id, obj_idx, sub_idx, command, data):
    global ecan_type
    ecan_id = cob_id.zfill(8)
    global ecan_dlc
    data = data.zfill(8)
    data_high = data[:4]
    data_low = data[4:]

    ecan_data = ecan_type + ecan_id + ecan_dlc + command + obj_idx[2:4] + obj_idx[0:2] + sub_idx + data_low[2:4] + data_low[:2] + data_high[2:4] + data_high[:2]
    # print(f"sand: {ecan_data[:2]} {ecan_data[2:10]} {ecan_data[10:12]} {ecan_data[12:14]} {ecan_data[14:16]} {ecan_data[16:18]} {ecan_data[18:20]} {ecan_data[20:22]} {ecan_data[22:24]} {ecan_data[24:26]} {ecan_data[26:]}")
    ecan_data = bytes.fromhex(ecan_data)
    client_socket.send(ecan_data)
    time.sleep(0.1)
    return recv_data()

def recv_data():
    try:
        data = client_socket.recv(1024)
        if not data:
            print(f"recv 없음")
        else:
            # print(f"recv: {data.hex()[:2]} {data.hex()[2:10]} {data.hex()[10:12]} {data.hex()[12:14]} {data.hex()[14:16]} {data.hex()[16:18]} {data.hex()[18:20]} {data.hex()[20:22]} {data.hex()[22:24]} {data.hex()[24:26]} {data.hex()[26:]} \n")
            return data.hex()

    except Exception as e:
        print(f"오류 발생: {e}")
        client_socket.close()

def make_data_form(data):
    if int(data) < 0:
        # 음수 10진수를 16진수로 변형
        return str(hex((data + (1 << 32))))[2:]
    return str(hex(int(data)))[2:].zfill(8)

def enable_motor():
    send_data("601", "6040", "00", "2b", "6")
    send_data("601", "6040", "00", "2b", "7")
    send_data("601", "6040", "00", "2b", "f")

def init_ecan():
    send_data("601", "6040", "00", "2b", "0")
    send_data("601", "6040", "00", "2b", "6")
    send_data("601", "6040", "00", "2b", "7")
    send_data("601", "6040", "00", "2b", "f")

def set_async():
    send_data("601","200f", "00", "2b", "0")

def stop_motor():
    send_data("601", "6040", "00", "2b", "0")

def set_motor_accel_time(set_time):
    # Set the motor Acceleration time
    # left motor
    send_data("601", "6083", "01", "23", make_data_form(set_time))
    # right motor
    send_data("601", "6083", "02", "23", make_data_form(set_time))

def set_motor_decel_time(set_time):
    # Set the motor Deceleration time
    # left motor
    send_data("601", "6084", "01", "23", make_data_form(set_time))
    # right motor
    send_data("601", "6084", "02", "23", make_data_form(set_time))

def check_encorder():
    global last_left_encoder, last_right_encoder
    # left encorder
    recv_data = send_data("601", "6064", "01", "40", "0")
    left_encorder = recv_data[-2:] + recv_data[-4:-2] + recv_data[-6:-4] + recv_data[-8:-6]
    # 문자열을 16진수로 변환 후 32bit signed int로 형변환 왼쪽 바퀴는 반대로 회전
    left_encorder = -ctypes.c_int32(int(left_encorder, 16)).value
    left_encorder_gap = left_encorder - last_left_encoder
    last_left_encoder = left_encorder

    # right encorder
    recv_data = send_data("601", "6064", "02", "40", "0")
    right_encorder = recv_data[-2:] + recv_data[-4:-2] + recv_data[-6:-4] + recv_data[-8:-6]
    # 문자열을 16진수로 변환 후 32bit signed int로 형변환
    right_encorder = ctypes.c_int32(int(right_encorder, 16)).value
    right_encorder_gap = right_encorder - last_right_encoder
    last_right_encoder = right_encorder

    # print(f"left : {left_encorder_gap}")
    # print(f"right: {right_encorder_gap}\n")

    return left_encorder_gap, right_encorder_gap

# while True:
#     cmd = input(" 1. Position Mode\n 2. Velocity Mode\n 3. Torque Mode\n 4. Custom Mode\n 5. STOP Motor\n 6. Enable Motor \n 7. Check Encorder \ncmd>>")
#     if cmd == "1":
#         position_mode()
#     elif cmd == "2":
#         velocity_mode()
#     elif cmd == "3":
#         torque_mode()
#     elif cmd == "4":
#         custom_mode()
#     elif cmd == "5":
#         stop_motor()
#     elif cmd == "6":
#         enable_motor()
#     elif cmd == "7":
#         check_encorder()
#     else:
#         print("잘못된 입력")



"""
ZLAC8015D
최대 장착 수 127
CAN bus 통신 속도 범위 25 ~ 1000kbps (기본값 500kbps)

ZLLG65ASM250

바퀴 지름 173mm     => 0.173m
바퀴 사이 거리 400mm => 0.4m


dw = 0.2


odom
move_per_deg = pow(173 / 2, 2) * pi / 360

"""


"""
Init

60186040002b00000000

60186040002b00000006

60186040002b00000007

60186040002b0000000f
"""
"""
Profile Velocity Mode Description

Set asynchronous control
6018200f002b00000000
Set synchronous control
6018200f002b00000001

Set profile velocity mode
60186060002f00000003

Set the left motor Acceleration time 100ms
60186083012300000064

Set the left motor Deceleration time 100ms
60186084012300000064

Set the right motor Acceleration time 100ms
60186083022300000064

Set the right motor Deceleration time 100ms
60186084022300000064

enable motor
60186040002b00000006

60186040002b00000007

60186040002b0000000f



Set the left motor Target speed 100rpm
601860ff012300000065

601860ff0223ffffff9c

Set the left & right motor Target speed 100rpm
601860ff032300640064


OD에 설정된 정보를 가지고 온다?
code 40은 OD에 저장된 데이터를 가지고 온다~
601860ff014000000000

idx와 s-idx는 OD의 index이다.

stop motor
60186040002b00000000

The motor stops and remains enabled
60186040002b0000010f

Motor enable (release emergency stop state)
60186040002b0000000f

04 00000601 08 2b 0f 20 00 00 00 00 00

04 00000601 08 2f 60 60 00 03 00 00 00

04 00000601 08 23 83 60 01 00 64 00 00

04 00000601 08 23 83 60 02 00 64 00 00

04 00000601 08 23 84 60 01 00 64 00 00

04 00000601 08 23 84 60 02 00 64 00 00

04 00000601 08 2b 40 60 00 06 00 00 00

04 00000601 08 2b 40 60 00 07 00 00 00

04 00000601 08 2b 40 60 00 0f 00 00 00

04 00000601 08 23 ff 60 03 00 64 00 64


1라디안은 180 / 원주율


엔코더 값 읽기
6064 01 40

회전량을 출력을 발생 시킨다??


왼쪽 바퀴 반시계 회전
4294831291

4294832448

4294833185

unsigned int -> signed int

1바퀴 엔코더 값: 16384 = 0x4000
1바퀴 0.173m

"""