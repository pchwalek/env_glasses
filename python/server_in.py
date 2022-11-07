import serial
from os.path import exists
import time
import _thread as thread
import threading
import queue
import struct as struct
from socket import gethostbyname
import socket

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

SERVER_HOST = socket.gethostname()
SERVER_PORT = 65435  # Port to listen on (non-privileged ports are > 1023)

def start_server():

    print("Starting server")

    msgQueue = queue.Queue(maxsize=20)

    try:
        print("Starting threads")
        server_thread = serverClass(2, "socket_thread", SERVER_HOST, SERVER_PORT, msgQueue)

        server_thread.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        print("Closing threads")
        server_thread.join()

class serverClass (threading.Thread):
    def __init__(self, threadID, name, host, port, queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.host = host
        self.port = port
        self.queue = queue

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            while True:
                conn, addr = s.accept()

                # flush queue
                while not self.queue.empty():
                    try:
                        self.queue.get(False)
                    except queue.Empty:
                        continue

                while True:
                    try:
                        # data = conn.recv(1024).decode().strip('][').split(', ')
                        data = conn.recv(1024).decode()
                        print(data)
                    except ConnectionResetError:
                        break

                # with conn:
                #     print(f"Connected by {addr}")
                #     while True:
                #         message = self.queue.get(block=True, timeout=None)
                #         try:
                #             conn.sendall(message.encode())
                #         except (ConnectionAbortedError, ConnectionResetError):
                #             print("conn aborted")
                #             break


if __name__ == '__main__':
    start_server()