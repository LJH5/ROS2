#!/usr/bin/env python3

import socket
import time


# Client TCP connect
HOST = "192.168.100.120"
PORT = 4001
TIMEOUT = 3
TIMESLEEP = 0.5

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))


# 1Byte = 8bit => 4bit / 4bit => 16진수 숫자 2개


while True:
    frame = input("cob-id / DLC / idx / s-idx / cmd / data_high / data_low \n=>")

    ecan_type = "04"
    cob_id = frame[:3]
    ecan_id = cob_id.zfill(8)
    ecan_dlc = frame[3:4].zfill(2)

    obj_idx = frame[4:8]
    sub_idx = frame[8:10]
    cmd = frame[10:12]
    if cmd == "40":
        data_high = "0000"
        data_low = "0000"
    else:
        data_high = frame[12:16]
        data_low = frame[16:20]

    ecan_data = ecan_type + ecan_id + ecan_dlc + cmd + obj_idx[2:4] + obj_idx[0:2] + sub_idx + data_low[2:4] + data_low[:2] + data_high[2:4] + data_high[:2]

    print(f"sand: {ecan_data[:2]} {ecan_data[2:10]} {ecan_data[10:12]} {ecan_data[12:14]} {ecan_data[14:16]} {ecan_data[16:18]} {ecan_data[18:20]} {ecan_data[20:22]} {ecan_data[22:24]} {ecan_data[24:26]} {ecan_data[26:]} \n")

    ecan_data = bytes.fromhex(ecan_data)
    client_socket.send(ecan_data)

    try:
        data = client_socket.recv(1024)
        if not data:
            print(f"recv 없음")
        else:
            print(f"recv: {data.hex()[:2]} {data.hex()[2:10]} {data.hex()[10:12]} {data.hex()[12:14]} {data.hex()[14:16]} {data.hex()[16:18]} {data.hex()[18:20]} {data.hex()[20:22]} {data.hex()[22:24]} {data.hex()[24:26]} {data.hex()[26:]} \n")
            pass

    except Exception as e:
        print(f"오류 발생: {e}")
        break

"""
ZLAC8015D
최대 장착 수 127
CAN bus 통신 속도 범위 25 ~ 1000kbps (기본값 500kbps)
"""

"""
Init

601860ff012300000065

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
60186040002b0000000f

60186040002b00000006

60186040002b00000007


Set the left motor Target speed 100rpm
601860ff012300000065

601860ff0223ffffff9c

OD에 설정된 정보를 가지고 온다?
cmd 40은 OD에 저장된 데이터를 가지고 온다~
601860ff014000000000

idx와 s-idx는 OD의 index이다.

stop motor
60186040002b0000010f

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
