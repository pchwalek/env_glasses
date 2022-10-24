import socket
from thermopile import *
from bme import *

import threading

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
IN_PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
OUT_PORT = 65433  # Port to send data out on

TEMPLE_TEMP_TRACK = 0

def unityParser(data):
    global TEMPLE_TEMP_TRACK
    out_data = []
    descriptor = data[0][1:-1]
    print(data)
    if (descriptor == 'Thermopile'):
        if (int(float(data[1])) == THERMOPLE_NOSE_TIP_ID):
            if(TEMPLE_TEMP_TRACK == 0):
                return out_data
            out_data.append("THERMOPILE_COG")
            out_data.append(str(TEMPLE_TEMP_TRACK - float(data[6])))
        elif (int(float(data[1])) == THERMOPLE_TEMPLE_MID_ADDR_ID):
            out_data.append("THERMOPILE_TEMPLE")
            out_data.append(data[6])
            TEMPLE_TEMP_TRACK = float(data[6])
    elif(descriptor == "SGP"):
        out_data.append("SGP_GAS")
        out_data.append(data[3])
        out_data.append(data[4])
    elif(descriptor == "BME"):
        sensor_ID = int(float(data[4]))
        if(sensor_ID == BSEC_IAQ):
            out_data.append("BME_IAQ")
            out_data.append(data[2])
        elif (sensor_ID == BSEC_CO2_EQ):
            out_data.append("BME_CO2_EQ")
            out_data.append(data[2])
        elif (sensor_ID == BSEC_BREATH_VOC_EQ):
            out_data.append("BME_VOC_EQ")
            out_data.append(data[2])
    elif(descriptor == "SHT45"):
        out_data.append("SHT45")
        out_data.append(data[1])
        out_data.append(data[2])
    elif(descriptor == "Lux"):
        out_data.append("Lux")
        out_data.append(data[1])
    elif (descriptor == "Blink"):
        return out_data

        # if(int(data[3]))
        # print(int(data[3]))
        #
        # out_data.append("SGP_GAS")
        # out_data.append(data[3])
        # out_data.append(data[4])


    return out_data

class unityMessage (threading.Thread):
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

                with conn:
                    print(f"Connected by {addr}")
                    while True:
                        message = self.queue.get(block=True, timeout=None)
                        try:
                            # print(message)
                            conn.sendall(message.encode())
                        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                            print("conn aborted")
                            break

while(True):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_in:
            while(True):
                try:
                    s_in.connect((HOST, IN_PORT))
                except ConnectionRefusedError:
                    print("ERROR: can't connect to python script")
                    continue
                break

            # s.sendall(b"Hello, world")
            msgQueue = queue.Queue(maxsize=20)

            unity_thread = unityMessage(1, "unity_thread", HOST, OUT_PORT, msgQueue)
            unity_thread.start()

            while True:
                try:
                    data = s_in.recv(1024).decode().strip('][').split(', ')
                except ConnectionResetError:
                    break
                # parse out data
                listMsg = unityParser(data)
                if(listMsg==[]):
                    continue
                strMsg = ','.join(listMsg)

                try:
                    msgQueue.put_nowait(json.dumps(strMsg))
                except queue.Full:
                    pass
            # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_out:
            #     s_out.bind((HOST, OUT_PORT))
            #     s_out.listen()
            #     while True:
            #         conn, addr = s_out.accept()
            #
            #         with conn:
            #             print(f"Connected by {addr}")
            #             while True:
            #                 data = s_in.recv(1024).decode().strip('][').split(', ')
            #
            #                 # parse out data
            #                 listMsg = unityParser(data)
            #                 strMsg = ','.join(listMsg)
            #                 print(listMsg)
            #                 # try:
            #                 #     conn.sendall(strMsg.encode())
            #                 # except (ConnectionAbortedError, ConnectionResetError):
            #                 #     print("conn aborted")
            #                 #     break
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        print("Closing threads")

        unity_thread.join()