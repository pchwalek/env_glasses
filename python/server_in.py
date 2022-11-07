import serial
from os.path import exists
import time
import _thread as thread
import threading
import queue
import struct as struct
from socket import gethostbyname
import socket

from airspecInfluxInterface import *

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

SERVER_HOST = socket.gethostname()
# SERVER_HOST = 'localhost'
SERVER_PORT = 65434  # Port to listen on (non-privileged ports are > 1023)

MAX_CONNECTIONS = 5

def start_server():

    print("Starting server")

    msgQueue = queue.Queue(maxsize=20)

    try:
        print("Starting threads")
        server_thread = serverClass(2, "socket_thread", SERVER_HOST, SERVER_PORT, msgQueue)
        influx_thread = serverLogger(3, "influx_logger", airspecDatabaseName, airspecKeys, msgQueue)

        influx_thread.start()
        server_thread.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        print("Closing threads")
        influx_thread.join()
        server_thread.join()

class serverClass (threading.Thread):
    def __init__(self, threadID, name, host, port, queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.host = host
        self.port = port
        self.queue = queue
        self.threadCount = 0

    def handleConnection(self, connection):
        while True:
            try:
                # data = conn.recv(1024).decode().strip('][').split(', ')
                data = connection.recv(1024).decode()
                if not data:
                    print("Connection issue. Restarting.")
                    break
                else:
                    self.queue.put(data)
            except ConnectionResetError:
                break
            except KeyboardInterrupt:
                connection.close()
                break
    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            while True:
                conn, addr = s.accept()
                print("Connection from: " + str(addr))
                if(self.threadCount < MAX_CONNECTIONS):
                    self.threadCount += 1
                    thread.start_new_thread(self.handleConnection, (conn,))
                else:
                    print(" number of connection exceeded! Max: " + str(MAX_CONNECTIONS))
                    conn.close()

                # flush queue
                while not self.queue.empty():
                    try:
                        self.queue.get(False)
                    except queue.Empty:
                        continue

                # while True:
                #     try:
                #         # data = conn.recv(1024).decode().strip('][').split(', ')
                #         data = conn.recv(1024).decode()
                #         if not data:
                #             print("Connection issue. Restarting.")
                #             break
                #         else:
                #             print(data)
                #     except ConnectionResetError:
                #         break
                #     except KeyboardInterrupt:
                #         s.close()
                #         break

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