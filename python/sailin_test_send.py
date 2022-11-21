from struct import *
import pandas as pd
import time
import socket
import queue
import json

# HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
HOST = "airspecs.media.mit.edu"
PORT = 64235  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_in:
    while True:
        try:
            print("Connecting to: " + HOST)
            print(socket.gethostbyname(HOST))
            s_in.connect((HOST, PORT))
            while True:
                s_in.sendall("test".encode())
                time.sleep(1)
        except OSError:
            print("Can't connect. Generated OSError.")
            continue
        except ConnectionRefusedError:
            print("ERROR: can't connect to python script")
            continue
        except KeyboardInterrupt:
            print("keyboard interrupt")
            s_in.close()
        break
