from struct import *
import pandas as pd
import time
import socket
import queue
import json

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

SPEC_PKT = 3
BME_PKT = 4
IMU_PKT = 6
THERMOPILE_PKT = 7
LUX_PKT = 8
MIC_PKT = 10
SHT_PKT = 11
SGP_PKT = 12
BLINK_PKT = 13

# packet information
headerStructType = 'BHIIIIIIII'
headerStructSize = calcsize(headerStructType)

SAVE_EVERY_X_SECS = 10

import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from collections import namedtuple
from datetime import datetime

class Sensor(namedtuple('Sensor', ['name', 'location', 'type', 'ms_from_start', 'value', 'timestamp'])):
    """
    Named structure - Sensor
    """
    pass

class SensorClass:
    def __init__(self, filepath, name="null", queue=0, header=[], structType="", influx_queue = []):
        self.name = name
        self.filepath = filepath
        if self.filepath == "":
            self.save_file_enable = 1
        else:
            self.save_file_enable = 0

        self.header = header.copy()
        self.header.append("epoch")
        self.df = pd.DataFrame(columns=self.header)
        self.structType = structType
        self.structSize = calcsize(structType)
        self.start_timestamp = int(time.time())
        self.last_save = 0
        self.queue = queue

        self.influx_queue = influx_queue

        self.saveIdx = 0
        self.firstSave = 0

        # socket communication specific
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def append_data(self, dataEntry):
        if (self.save_file_enable == 1):
            self.df.loc[len(self.df)] = dataEntry
        try:
            self.queue.put_nowait(json.dumps([self.name]+dataEntry))
        except queue.Full:
            pass

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
            # self.save_file()

            if(self.influx_queue != []):
                self.send_to_influx(dict(zip(self.header, unpacked_pkt)))

        if (self.save_file_enable == 1):
            if( (time.time() - self.last_save) > SAVE_EVERY_X_SECS):
                self.save_file()
                self.last_save = time.time()


    def connect_to_socket(self, host='', port=65432):
        self.s.connect((host,port))

    # below function needs to be implemented by inherited class
    def send_to_influx(self, pkt_dict):
        return

    # def send_to_influx(self, type, value, mcu_timestamp):
    #     sensor = Sensor(name=self.name,
    #         location = "boston",
    #         type = type,
    #         ms_from_start = mcu_timestamp,
    #         value = value,
    #         timestamp = datetime.utcnow())
    #
    #     self.influx_queue.put(sensor)
    #     return