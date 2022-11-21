from struct import *
import pandas as pd
import time
import socket
import queue
import json

# HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
HOST = socket.gethostname()
PORT = 64235  # Port to listen on (non-privileged ports are > 1023)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()

    while True:
        conn, addr = s.accept()

        with conn:
            print(f"Connected by {addr}")
            while True:
                try:
                    data = conn.recv(1024)
                    print(data)
                except (ConnectionAbortedError, ConnectionResetError):
                    print("conn aborted")
                    break