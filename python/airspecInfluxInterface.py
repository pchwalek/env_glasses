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

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

airspecDatabaseName = 'airspec'
airspecKeys = []

class serverLogger (threading.Thread):
    def __init__(self, threadID, name, database_name, keys, queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.database_name = database_name
        self.keys = keys
        self.queue = queue

    def run(self):
        # flush queue
        while not self.queue.empty():
            try:
                self.queue.get(False)
            except queue.Empty:
                continue

        while True:
            message = self.queue.get(block=True, timeout=None)
            try:
                print(message)
                # conn.sendall(message.encode())
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                print("conn aborted")
                break



def start_server_logger():
    client = InfluxDBClient(host='localhost', port=8086)

    # check if database already exists and if not, create it
    database_exist = False
    current_databases = client.get_list_database()
    for database in current_databases:
        if database == database_name:
            database_exist = True
            break
    if not database_exist:
        client.create_database(database_name)
    client.switch_database(database_name)

if __name__ == '__main__':
    start_server_logger()