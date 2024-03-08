import socket
import time


# Client TCP connect
HOST = ""
PORT = 
TIMEOUT = 3

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

# type은 command
def gen_command(_cob,_len,_index, _type, _data1, _data2): # 601,8,60ff03,2f,2064,0000

    data1 =''
    data2 =''
    ret = ''
    _len = '0'+_len
    if _data1 and _data2:
        data1 = tohex(_data1,16)
        data2 = tohex(_data2,16)
    elif _data2 :
        if _type[1] != '3':
            data2 = tohex(_data2,16)
        else :
            data2 = tohex(_data2,32)
    elif _data1 :
        data1 = tohex(_data1,16)
        data2 = tohex(0,16)

    else:
        data1 = tohex(0,32)

    high, low = alignment_data(data1[2:]+data2[2:],_type)

    ret = '0400000'+_cob+_len+_type+alignment_Addr(_index)+low+high+'\r'

    ret = bytes.fromhex(ret)
    return ret

def tohex(v, b):
    conv = (v &(2 ** b - 1))
    idx = int(2+(b/4))
    ret=hex(conv)
    add = ''

    if len(ret)<idx:
        idx = idx - len(ret)
        for i in range(idx):
            add += '0'
        ret = ret[:2]+add+ret[2:]
    return ret

def alignment_Addr(_addr):
    return _addr[2:4]+_addr[0:2]+_addr[4:6]

def alignment_data(_data, _size):
    high = '0000'
    low='0000'
    if(_size=='40'):
        high = '0000'
        low = '0000'
    elif(_size[1] == '3' or _size[1] == '0' ):
        # print(_data,_size)
        high = _data[2:4]+_data[0:2]
        low = _data[6:8]+_data[4:6]
    else:
        high = '0000'
        low = _data[2:4]+_data[0:2]
    return high, low

# eCAN
# TYPE / ID          / DLC / DATA
#  04  / 00 00 06 01 / 08  / 43 ff 60 03 00 00 10 08

# CANOpen
#     / COB-ID      /    / cmd / obj idx / obj sub-idx / Data
#  04 / 00 00 06 01 / 08 / 43  / ff 60   / 03          / 00 00 10 08

# send_data = gen_command('601','8','604000','2b',0x004f, 0x0000)

while True:
    # 60186040002b004f0000
    command = input()
    send_data = gen_command(command[:3], command[3:4], command[4:10], command[10:12], int(command[12:16], 16), int(command[16:],16))
    client_socket.send(send_data)

    try:
        data = client_socket.recv(1024)
        if not data:
            print(f"data 없음")
        else:
            print(f"data: {data.hex()[:2]} {data.hex()[2:10]} {data.hex()[10:12]} {data.hex()[12:14]} {data.hex()[14:16]} {data.hex()[16:18]} {data.hex()[18:20]} {data.hex()[20:22]} {data.hex()[22:24]}")
    except Exception as e:
        print(f"오류 발생: {e}")
        client_socket.close()
        break

client_socket.close()
