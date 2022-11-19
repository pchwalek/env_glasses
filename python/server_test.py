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

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
SERVER_HOST = "airspecs.media.mit.edu"
# SERVER_HOST = "localhost"
SERVER_PORT = 65435  # Port to listen on (non-privileged ports are > 1023)
# HOST = gethostbyname('')
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_in:
    while True:
        try:
            print("Connecting to: " + SERVER_HOST)
            print(gethostbyname(SERVER_HOST))
            s_in.connect((SERVER_HOST, SERVER_PORT))
        except ConnectionRefusedError:
            print("ERROR: can't connect to python script")
            continue
        break
    while True:
        s_in.sendall("test".encode())
        time.sleep(1)
    s_in.close()