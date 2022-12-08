from influxdb import InfluxDBClient
import serial
from os.path import exists
import time
import _thread as thread
import threading
import queue
import struct as struct
from socket import gethostbyname
import socket
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from collections import namedtuple

from thermopile import *
from sht4x import *
from sgp import *
from bme import *
from lux import *
from spec import *
from blink import *
from imu import *

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

airspecDatabaseName = 'airspec'
airspecRawKeys = []

INFLUX_HOST = 'localhost'
INFLUX_PORT = 8086

DISABLE_FILE_SAVING = ""

DATA_DIR = "data/"
filename_noext = 'logfile'
filename_prefix = ''
filename_extension = ".csv"
temp_header = "temp_1, temp_2, temp_3, ambient_temp, ardu_millis, epoch\n"
hr_header = "heart_rate, epoch\n"

class serverLogger (threading.Thread):
    def __init__(self, threadID, name, database_name, keys, queue, influx_queue, unity_queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.database_name = database_name
        self.keys = keys
        self.queue = queue

        self.unity_queue = unity_queue

        self.influx_queue = influx_queue

    def run(self):
        thermopile = Thermopile(DISABLE_FILE_SAVING, self.unity_queue, "Thermopile", self.influx_queue)
        sht45 = SHT4X(DISABLE_FILE_SAVING, self.unity_queue, "SHT45", self.influx_queue)
        sgp = SGP(DISABLE_FILE_SAVING, self.unity_queue, "SGP", self.influx_queue)
        spec = Spec(DISABLE_FILE_SAVING, self.unity_queue, "Spec", self.influx_queue)
        lux = Lux(DISABLE_FILE_SAVING, self.unity_queue, "Lux", self.influx_queue)
        bme = BME(DISABLE_FILE_SAVING, self.unity_queue, "BME", self.influx_queue)
        blink = Blink(DISABLE_FILE_SAVING, self.unity_queue, "Blink", self.influx_queue, store_raw=1)
        # imu = IMU(DATA_DIR, self.queue, "IMU", parquet=True)
        imu = IMU(DISABLE_FILE_SAVING, self.unity_queue, "IMU", self.influx_queue, store_raw=1)

        therm_pkt = 0
        sht_pkt = 0
        sgp_pkt = 0
        spec_pkt = 0
        lux_pkt = 0
        bme_pkt = 0
        blink_pkt = 0
        imu_pkt = 0

        error_header_unpack = 0

        # flush queue
        while not self.queue.empty():
            try:
                self.queue.get(False)
            except queue.Empty:
                continue

        while True:
            ser_string = self.queue.get(block=True, timeout=None)
            try:
                # print("Server Logger")

                # print(ser_string)
                # (1) parse message
                try:
                    systemID, pktType, pktID, msFromStart, epoch, payloadLen, r0, r1, r2, r3, r4 = unpack(headerStructType, bytes.fromhex(ser_string[0:headerStructSize*2].decode("utf-8")))
                except (ValueError, struct.error):
                    print('ERROR: error in unpacking header')
                    error_header_unpack += 1
                    print(ser_string[0:headerStructSize * 2].decode("utf-8"))
                    continue
                except BaseException as err:
                    print(f"Unexpected {err=}, {type(err)=}")
                    raise
                # print("pktheader")
                # print(pktType)
                # print(pktID)
                # print(payloadLen)
                # (2) if message is not IMU or Blink data, send to InfluxDB
                if (THERMOPILE_PKT == pktType):
                    # print("thermopile packet received: " + str(pktID))
                    therm_pkt += 1
                    number_of_packed_packets = int(payloadLen / thermopileStructSize)
                    thermopile.unpack_compressed_packet(ser_string, number_of_packed_packets, sysID=systemID)
                elif (SHT_PKT == pktType):
                    # print("sht packet received: " + str(pktID))
                    sht_pkt += 1
                    number_of_packed_packets = int(payloadLen / shtStructSize)
                    sht45.unpack_compressed_packet(ser_string, number_of_packed_packets, sysID=systemID)
                elif (SPEC_PKT == pktType):
                    # print("sgp packet received: " + str(pktID))
                    spec_pkt += 1
                    number_of_packed_packets = int(payloadLen / specStructSize)
                    spec.unpack_compressed_packet(ser_string, number_of_packed_packets, sysID=systemID)
                elif (BME_PKT == pktType):
                    # print("bme packet received: " + str(pktID))
                    bme_pkt += 1
                    number_of_packed_packets = int(payloadLen / bmeStructSize)
                    bme.unpack_compressed_packet(ser_string, number_of_packed_packets, sysID=systemID)
                elif (LUX_PKT == pktType):
                    # print("lux packet received: " + str(pktID))
                    lux_pkt += 1
                    number_of_packed_packets = int(payloadLen / luxStructSize)
                    lux.unpack_compressed_packet(ser_string, number_of_packed_packets, sysID=systemID)
                elif (SGP_PKT == pktType):
                    # print("lux packet received: " + str(pktID))
                    sgp_pkt += 1
                    number_of_packed_packets = int(payloadLen / sgpStructSize)
                    sgp.unpack_compressed_packet(ser_string, number_of_packed_packets, sysID=systemID)

                # (3) if message is IMU or Blink data, save via compressed format
                elif (BLINK_PKT == pktType):
                    # print("lux packet received: " + str(pktID))
                    blink_pkt += 1
                    number_of_packed_packets = int(payloadLen / blinkStructSize)
                    blink.unpack_compressed_packet(ser_string, number_of_packed_packets, msFromStart, pktID,
                                                   sampleRate=r1, diodeSaturated=r0, sysID=systemID)
                elif (IMU_PKT == pktType):
                    imu_pkt += 1
                    # print(" imu payload len: " + str(payloadLen))
                    # print(" imu struct size: " + str(imuStructSize))
                    number_of_packed_packets = int(payloadLen / imuStructSize)
                    imu.unpack_compressed_packet(ser_string, number_of_packed_packets, msFromStart, pktID, sysID=systemID)

                # print("THERM: " + str(therm_pkt) + "\t" +
                #       "SHT: " + str(sht_pkt) + "\t" +
                #       "SGP: " + str(sgp_pkt) + "\t" +
                #       "SPEC: " + str(spec_pkt) + "\t" +
                #       "LUX: " + str(lux_pkt) + "\t" +
                #       "BME: " + str(bme_pkt) + "\t" +
                #       "BLINK: " + str(blink_pkt) + "\t" +
                #       "IMU: " + str(imu_pkt) + "\t" +
                #       "errors: " + str(error_header_unpack))


                # conn.sendall(message.encode())
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                print("conn aborted")
                break

class influxDBLogger (threading.Thread):
    def __init__(self, threadID, name, url, token, org, bucket, database_name, queue):
        threading.Thread.__init__(self)
        self.data = []
        self.client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        self.database_name = database_name

        self.bucket = bucket
        self.org = org
        # checkServerExist(self.client, self.database_name)
        self.queue = queue
    def run(self):
        while True:
            message = self.queue.get(block=True, timeout=None)
            # try:
            #     message = message.strip('][').split(', ')
            # except AttributeError:
            #     print("ATTRIBUTE ERROR")
            #     print(message)
            # message[0] = message[0][1:-1]
            # for num in range(len(message)):
            #     if num == 0:
            #         continue
            #     else:
            #         message[num] = float(message[num])
            # print("INFLUXDB THREAD: received message")
            # print(message)
            # self.data.append("{measurement},UID={UID},value={value},timestamp={timestamp}"
            #             .format(measurement=message[0], # measurement name
            #                     UID=message[1], # measurement UID
            #                     value=message[2], # measurement value
            #                     timestamp=message[3] ))# measurement_timestamp
            #
            # ref: https://docs.influxdata.com/influxdb/v1.8/write_protocols/line_protocol_tutorial/#:~:text=The%20minimum%20valid%20timestamp%20is,that%20timestamps%20have%20nanosecond%20precision.
            # print("influx entry")
            for i in range(len(message)):
                # print(message[i])
                # print("influx entry")
                try:
                    self.write_api.write(self.bucket, self.org, message[i])
                except influxdb_client.rest.ApiException:
                    print(" ERROR: exception raised when adding the below to InfluxDB")
                    print("   " + message[i])
                # self.write_api.write(bucket=self.bucket, record=message[i])

            # self.data = []

def start_server_logger(client):
    # client = InfluxDBClient(host='localhost', port=8086)

    # check if database already exists and if not, create it
    # database_exist = False
    # current_databases = client.get_list_database()
    # print(current_databases)
    # for database in current_databases:
    #     if database == database_name:
    #         database_exist = True
    #         break
    # if not database_exist:
    #     client.create_database(database_name)
    # client.switch_database(database_name)

    print("start_server_logger called: do nothing")

if __name__ == '__main__':
    start_server_logger()