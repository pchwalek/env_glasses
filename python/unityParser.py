import socket
from thermopile import *

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
IN_PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
OUT_PORT = 65433  # Port to send data out on


def unityParser(data):
    out_data = []

    if (data[0] == "thermopile"):
        if (data[1] == THERMOPLE_NOSE_TIP_ID):
            out_data.append("THERMOPILE_NOSE")
            out_data.append(data[5])
        elif (data[1] == THERMOPLE_TEMPLE_MID_ADDR_ID):
            out_data.append("THERMOPILE_TEMPLE")
            out_data.append(data[5])

    return out_data


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_in:
    s_in.connect((HOST, IN_PORT))
    # s.sendall(b"Hello, world")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_out:
        s_out.bind((HOST, OUT_PORT))
        s_out.listen()
        while True:
            conn, addr = s_out.accept()

            with conn:
                print(f"Connected by {addr}")
                while True:
                    data = s_in.recv(1024)

                    # parse out data
                    message = unityParser(data)

                    try:
                        conn.sendall(message.encode())
                    except (ConnectionAbortedError, ConnectionResetError):
                        print("conn aborted")
                        break
