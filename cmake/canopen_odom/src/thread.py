import threading
import time

# define work
def work(_cnt, _delay, _name):
    print(f"{_name} got {_cnt} works")

    for i in range(_cnt):
        print(f"{_name} finished {i+1} works")
        time.sleep(_delay)

    print(f"{_name} leaved work")

# common thread
common_thread = threading.Thread(target=work, args=(5, 1, 'common'))
common_thread.start()

# demon thread
"""
백그라운드 작업 수행, 다른 스레드 모두 종료 시 강제 종료, 낮은 우선 순위
"""
daemon_thread = threading.Thread(target=work, args=(5, 2, 'daemon'))
daemon_thread.daemon = True
daemon_thread.start()

# main thread
work(3, 0.5, 'main')