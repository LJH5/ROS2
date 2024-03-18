#!/usr/bin/env python3

import socket
import time
from math import pi, pow
import ZL_LIB


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

# 1Byte = 8bit => 4bit / 4bit => 16진수 숫자 2개

### define function ######################################
def position_mode():
    print("###### Position Mode ######")

    cob_id = "601"

    # init
    init_ecan()

    # set profile postion
    send_data(cob_id, "6060", "00", "2f", "1")

    time_set = "100"
    # Set the motor Acceleration time
    # left motor
    send_data(cob_id, "6083", "01", "23", time_set)
    # right motor
    send_data(cob_id, "6083", "02", "23", time_set)

    # Set the motor Deceleration time
    # left motor
    send_data(cob_id, "6084", "01", "23", time_set)
    # right motor
    send_data(cob_id, "6084", "02", "23", time_set)

    # Set the motor Maximum speed
    speed = "60"
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
    obj_idx = "6060"
    sub_idx = "00"
    command = ""
    data_low = "0002"
    pass

def torque_mode():
    pass

def custom_mode():
    obj_idx = input("obj_idx: ")
    sub_idx = input("sub_idx: ")
    command = input("command: ")
    pass

def send_data(cob_id, obj_idx, sub_idx, command, data):
    global ecan_type
    ecan_id = cob_id.zfill(8)
    global ecan_dlc
    data = data.zfill(8)
    data_high = data[:4]
    data_low = data[4:]

    ecan_data = ecan_type + ecan_id + ecan_dlc + command + obj_idx[2:4] + obj_idx[0:2] + sub_idx + data_low[2:4] + data_low[:2] + data_high[2:4] + data_high[:2]
    print(f"sand: {ecan_data[:2]} {ecan_data[2:10]} {ecan_data[10:12]} {ecan_data[12:14]} {ecan_data[14:16]} {ecan_data[16:18]} {ecan_data[18:20]} {ecan_data[20:22]} {ecan_data[22:24]} {ecan_data[24:26]} {ecan_data[26:]}")
    ecan_data = bytes.fromhex(ecan_data)
    client_socket.send(ecan_data)
    time.sleep(0.1)
    recv_data()

def recv_data():
    try:
        data = client_socket.recv(1024)
        if not data:
            print(f"recv 없음")
        else:
            print(f"recv: {data.hex()[:2]} {data.hex()[2:10]} {data.hex()[10:12]} {data.hex()[12:14]} {data.hex()[14:16]} {data.hex()[16:18]} {data.hex()[18:20]} {data.hex()[20:22]} {data.hex()[22:24]} {data.hex()[24:26]} {data.hex()[26:]} \n")
            pass

    except Exception as e:
        print(f"오류 발생: {e}")
        client_socket.close()

def make_data_form(data):
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

while True:
    cmd = input(" 1. Position Mode\n 2. Velocity Mode\n 3. Torque Mode\n 4. Custom Mode\n 5. off\ncmd>>")
    if cmd == "5":
        send_data("601", "6040", "00", "2b", "0")
    elif cmd == "4":
        custom_mode()
    else:
        position_mode()
        # if command != "40":         # 40(READ)은 data가 00000000 고정
        #     data_high = cmd[12:16]
        #     data_low = cmd[16:20]


"""
ZLAC8015D
최대 장착 수 127
CAN bus 통신 속도 범위 25 ~ 1000kbps (기본값 500kbps)

ZLLG65ASM250

바퀴 지름 173mm
바퀴 사이 거리 400cm

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

"""
