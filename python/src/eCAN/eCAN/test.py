import socket
import threading
import queue
import time
from tkinter.constants import ON
from tkinter.tix import MAX
import my_func as my

MAX_BUF = 20

class Connect_Client():
    def __init__ (self, *args, **kwargs):
        self.sock = None
        self.host = kwargs['HOST']
        self.port = kwargs['PORT']
        self.rxque = args[0]
        self.txque = args[1]
        self.txchk = args[2]
        self.rxthread= None
        self.txthread= None

    def Connect(self, sw):
        if sw == ON:
            # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.sock:
            self.sock =  socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
            self.sock.connect((self.host,self.port))
            print("open sock:",self.sock)

        else :
            # self.txthread.close()
            # self.txthread.join()
            self.rxthread.close()
            # self.rxthread.join()
            self.sock.close();
            # self.sock=None

        return self.sock
    def Open_thread(self):

        self.rxthread = rxThread(self.sock, self.rxque, self.txque, self.txchk)
        self.rxthread.setDaemon(True)
        self.rxthread.start()
        # self.txthread = txThread(self.sock, self.rxque, self.txque, self.txchk)
        # self.txthread.setDaemon(True)
        # self.txthread.start()
        pass

class rxThread(threading.Thread):
    def __init__(self, sock,*args):
        threading.Thread.__init__(self)
        self.sock = sock
        print("rx sock:",self.sock)
        self.rx_que = args[0]
        self.tx_que = args[1]
        self.tx_chk = args[2]
        self.is_run = True
        self.recv_dat=""
        self.queue = queue.Queue()
        self.tx_cnt=0
        self.time_out = 0.01
        self.new_time =0
        self.last_time=0
        self.send_thread = threading.Thread(target=self.sendThread)
        self.send_thread.setDaemon(True)
        self.send_thread.start()
    def run(self):
        while self.is_run == True:
            data = self.sock.recv(1024)
            s=''
            for d in data:
                s += my.tohex(d,8)[2:]
            # print("rx_thread",s)
            self.rx_que.append(data)

            # self.sock.close();
            pass
    def sendThread(self):
        while self.is_run == True:
            # self.new_time = time.time()
            # if(self.new_time - self.last_time) >= self.time_out:
            if(len(self.tx_que)):
                # print("tx_thread",self.tx_que)
                self.sock.send(self.tx_que[0])
                self.tx_chk.append(self.tx_que[0])
                del self.tx_que[0]
                # self.last_time = self.new_time
            if len(self.tx_que)>MAX_BUF:
                cnt = len(self.tx_que)-MAX_BUF
                del self.tx_que[0:cnt]
            if len(self.tx_chk)>MAX_BUF:
                cnt = len(self.tx_chk)-MAX_BUF
                del self.tx_chk[0:cnt]
            time.sleep(self.time_out)
            pass

    def close(self):
        self.is_run = False

class txThread(threading.Thread):
    def __init__(self, sock,*args):
        threading.Thread.__init__(self)
        self.sock = sock
        print("tx sock:",self.sock)
        self.rx_que = args[0]
        self.tx_que = args[1]
        self.tx_chk = args[2]
        self.is_run = True
        self.recv_dat=""
        self.queue = queue.Queue()
        self.tx_cnt=0
        self.time_out = 0.02
        self.new_time =0
        self.last_time=0

    def run(self):
        while self.is_run == True:
            # time.sleep(0.00001)
            self.new_time = time.time()
            if(self.new_time - self.last_time) >= self.time_out:
                if(len(self.tx_que)):
                    # print("tx_thread",self.tx_que)
                    self.sock.send(self.tx_que[0])
                    self.tx_chk.append(self.tx_que[0])
                    del self.tx_que[0]
                self.last_time = self.new_time
            pass
    def close(self):
        self.is_run = False
