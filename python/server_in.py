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
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from collections import namedtuple
from datetime import datetime

MAX_RETRY_UPON_DISCONNECT = 1

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

# SERVER_HOST = socket.gethostname()
SERVER_HOST = 'localhost'
SERVER_PORT = 65434  # Port to listen on (non-privileged ports are > 1023)

MAX_CONNECTIONS = 5

INFLUX_TOKEN = "AFpAiw6t6d6-xugqcptMSy9_D8DB0WwLxW1IeNt7V0yN__hY5hRR8gOSAJfLDvjn21QUcebmQam--sgk2_dJDQ=="
# INFLUX_TOKEN = '7qVtGuNwum4qbsHhjDm_H2tUf3aeaEwc3uuInvStC6fcsMIefzsSZ4mOODfx2vQU_5_SOIKIy5Sxe_5OHv1HDQ=='
def checkServerExist(client, database_name):
    # check if database already exists and if not, create it
    database_exist = False

    bucket = "<BUCKET>"

    write_api = client.write_api(write_options=SYNCHRONOUS)

    for value in range(5):
        point = (
            Point("measurement1")
                .tag("tagname1", "tagvalue1")
                .field("field1", value)
        )
        write_api.write(bucket=bucket, org="media_lab", record=point)
        time.sleep(1)  # separate points by 1 second

    current_databases = client.get_list_database()
    for database in current_databases:
        if database == database_name:
            database_exist = True
            break
    if not database_exist:
        client.create_database(database_name)
    client.switch_database(database_name)

class Sensor(namedtuple('Sensor', ['name', 'location', 'version', 'value', 'timestamp'])):
    """
    Named structure - Sensor
    """
    pass

def start_server():

    print("Starting server")

    msgQueue = queue.Queue(maxsize=20)

    unityQueue = queue.Queue(maxsize=20)

    influxQueue = queue.Queue(maxsize=20)

    # client = influxdb_client.InfluxDBClient(url='localhost', port=8086, token=INFLUX_TOKEN)
    # client = InfluxDBClient(host='localhost', port=8086, username='airspec', password='airspec4life!')
    INFLUX_ORG = "media_lab"
    # url = "https://us-west-2-1.aws.cloud2.influxdata.com/"
    # url = "localhost"
    INFLUX_URL = "http://localhost:8086"

    client = influxdb_client.InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG, debug=True)

    bucket = "airspec"

    write_api = client.write_api(write_options=SYNCHRONOUS)
    water_level = 1

    data = []
    pkt_dict = {}
    pkt_dict["sensor_id"] = 1
    pkt_dict["signal"] = 1300
    pkt_dict["accuracy"] = 1
    pkt_dict["timestamp_ns"] = 113

    # for i in range(5):
    #     data.append(
    #         "{measurement},sensor_id={sensor_id},id={id},accuracy={accuracy},timestamp_ns={timestamp_ns}i signal={signal}"
    #         .format(measurement="bme_test",
    #                 sensor_id=pkt_dict["sensor_id"],
    #                 id=1,
    #                 signal=pkt_dict["signal"],
    #                 accuracy=pkt_dict["accuracy"],
    #                 timestamp_ns=pkt_dict["timestamp_ns"]))
    #     print(data)
    #     write_api.write(bucket, INFLUX_ORG, data)
    #     time.sleep(1)
    #     data = []
    #     pkt_dict["signal"] += 1
    # return

    # for value in range(5):
    #     # point = (
    #     #     Point("measurement1")
    #     #         .tag("tagname2", "tagvalue2")
    #     #         .field("field1", value)
    #     # )
    #
    #     sensor = Sensor(name="sensor_pt859",
    #         location = "warehouse_125",
    #         version = "2021.06.05.5874",
    #         value = water_level,
    #         timestamp = datetime.utcnow())
    #
    #     write_api.write(bucket="airspec",
    #                     record=sensor,
    #                     record_measurement_key="name",
    #                     record_time_key="timestamp",
    #                     record_tag_keys=["location", "version"],
    #                     record_field_keys=["value"])
    #     water_level += 1
    #     # point = Point.from_dict(dict_structure, WritePrecision.NS)
    #     # write_api.write(bucket=bucket, org="media_lab", record=point)
    #     time.sleep(1)  # separate points by 1 second

    # start_server_logger(client)
    try:
        print("Starting threads")
        # thread to grab data from external source via socket
        server_thread = serverClass(2, "socket_thread", SERVER_HOST, SERVER_PORT, msgQueue)
        # thread to parse data
        server_parser_thread = serverLogger(3, "server_parser", airspecDatabaseName, airspecRawKeys, msgQueue, influxQueue, unityQueue)
        # thread to push to influxDB
        influx_thread = influxDBLogger(4, "influx_logger", INFLUX_URL, INFLUX_TOKEN, INFLUX_ORG, bucket, airspecDatabaseName, influxQueue)

        server_parser_thread.start()
        server_thread.start()
        influx_thread.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        print("Closing threads")
        server_parser_thread.join()
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

        self.retry_attempts = 0

    def handleConnection(self, connection):
        while True:
            try:
                # data = conn.recv(1024).decode().strip('][').split(', ')
                data = connection.recv(1024)
                if not data:
                    # print("Empty packet received. Retrying")
                    # time.sleep(1)
                    # data = connection.recv(1024)
                    # if not data:
                    #     print("Another empty bracket. Closing connection.")
                    break
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

                # # flush queue
                # while not self.queue.empty():
                #     try:
                #         self.queue.get(False)
                #     except queue.Empty:
                #         continue

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