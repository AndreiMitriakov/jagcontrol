#!/usr/bin/env python3

import socket
import threading
import time

class Server (threading.Thread):
    def __init__(self, host, port):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.s = None

    def run(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('strt')
        self.s.bind((self.host, self.port))
        self.s.listen(1)
        while True:
            conn, addr = self.s.accept()
            with conn:
                conn.settimeout(1.0)
                print('conn:', addr)
                try:
                    while True:
                        print('wait')
                        data = conn.recv(1024).decode("utf-8")
                        if not data:
                            break
                        print('recv:', data)
                except Exception as e:
                    print('tout')
        self.s.close()
        self.s = None

    def __del__(self):
        print("delt")
        if self.s != None:
            print("clos")
            self.s.close()


if __name__ == '__main__':
    host = "127.0.0.1"
    ports = [10001, 10002]
    threads = []
    for port in ports:
        threads.append(Server(host, port))
    print('Threads created')
    for i, thread in enumerate(threads):
        print('Thread {} joined'.format(i))
        thread.start()
