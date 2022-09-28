from struct import *
import pandas as pd
import time
import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

SPEC_PKT = 3
BME_PKT = 4
THERMOPILE_PKT = 7
LUX_PKT = 8
MIC_PKT = 10
SHT_PKT = 11
SGP_PKT = 12

# packet information
headerStructType = 'BHIIIIIIII'
headerStructSize = calcsize(headerStructType)

SAVE_EVERY_X_SECS = 30

class SensorClass:
    def __init__(self, filepath, name="null", header=[], structType=""):
        self.name = name
        self.filepath = filepath
        self.header = header
        self.header.append("epoch")
        self.df = pd.DataFrame(columns=self.header)
        self.structType = structType
        self.structSize = calcsize(structType)
        self.start_timestamp = int(time.time())
        self.last_save = 0

        # socket communication specific
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def append_data(self, dataEntry):
        self.df.loc[len(self.df)] = dataEntry

    def save_file(self):
        self.df.to_csv(self.filepath + self.name + "_" + str(self.start_timestamp) + ".csv")

    def unpack_compressed_packet(self, data, pkt_cnt):
        time_recv = time.time()
        for idx in range(pkt_cnt):
            unpacked_pkt = list(unpack(self.structType,
                          bytes.fromhex(data[(headerStructSize * 2) + idx * (self.structSize * 2):
                                                   (headerStructSize * 2) + (idx + 1) * (self.structSize * 2)].decode("utf-8"))))
            unpacked_pkt.append(time_recv)
            self.append_data(unpacked_pkt)

        if( (time.time() - self.last_save) > SAVE_EVERY_X_SECS):
            self.save_file()

    def connect_to_socket(self, host='', port=65432):
        self.s.connect((host,port))