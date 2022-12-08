import serial
from os.path import exists
import time
import _thread as thread
import threading
import queue
import struct as struct
from socket import gethostbyname
import ctypes

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

# from sensorClass import *
from thermopile import *
from sht4x import *
from sgp import *
from bme import *
from lux import *
from spec import *
from blink import *
from imu import *

# airSpec_serial = '/dev/cu.usbserial-014A1C56'
airSpec_serial = '/dev/cu.usbserial-01D9209A'

# gasSpec_serial = 'COM4'

DATA_DIR = "data/"
filename_noext = 'logfile'
filename_prefix = ''
filename_extension = ".csv"
temp_header = "temp_1, temp_2, temp_3, ambient_temp, ardu_millis, epoch\n"
hr_header = "heart_rate, epoch\n"

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
# SERVER_HOST = "airspecs.media.mit.edu"
SERVER_HOST = "localhost"
SERVER_PORT = 65434  # Port to listen on (non-privileged ports are > 1023)
# HOST = gethostbyname('')
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

def start_experiment():

    print("Starting experiment")
    print(" connecting to the following server: "+ SERVER_HOST + \
          " on port " + str(SERVER_PORT))

    print("Header size of expected packet: " + str(headerStructSize))
    msgQueue = queue.Queue(maxsize=20)

    try:
        ''' note: the below threads can be combined into one but having them
        separated gives us the future opportunity to apply any parsing '''

        print("Starting threads")
        ''' this thread just listens for data coming from the ESP32 dongle and
            forwards it to the serverLogger thread '''
        serialListener = logSensor(1, "ardu_thread", temp_header, airSpec_serial, filename_prefix+"_temp"+filename_extension, msgQueue)

        # socket_thread = socketMessage(2, "socket_thread", HOST, PORT, msgQueue)

        ''' this thread connects to the server and forwards the data from the 
            ESP32 dongle '''
        client_thread = serverLogger(2, "socket_thread", SERVER_HOST, SERVER_PORT, msgQueue)

        # socket_thread.start()
        client_thread.start()
        serialListener.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        print("Closing threads")

        serialListener.join()
        client_thread.join()
        # socket_thread.join()

class socketMessage (threading.Thread):
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
                            conn.sendall(message.encode())
                        except (ConnectionAbortedError, ConnectionResetError):
                            print("conn aborted")
                            break

class serverLogger (threading.Thread):
    def __init__(self, threadID, name, host, port, queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.host = host
        self.port = port
        self.queue = queue

    def run(self):
        # s.sendall(b"Hello, world")
        # msgQueue = queue.Queue(maxsize=20)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_in:
                while True:
                    try:
                        print("Connecting to server at: " + self.host)
                        print(gethostbyname(self.host))
                        s_in.connect((self.host, self.port))
                    except OSError:
                        print("Can't connect to server! Generated OSError. Retrying in: ",end =" ")
                        time.sleep(1)
                        print("5.. ",end =" ")
                        time.sleep(1)
                        print("4.. ", end=" ")
                        time.sleep(1)
                        print("3.. ", end=" ")
                        time.sleep(1)
                        print("2.. ", end=" ")
                        time.sleep(1)
                        print("1.. ")
                        time.sleep(1)
                        continue
                    except ConnectionRefusedError:
                        print("ERROR: can't connect to python script")
                        continue
                    break

                # flush queue
                while not self.queue.empty():
                    try:
                        self.queue.get(False)
                    except queue.Empty:
                        continue

                while True:
                    try:
                        message = self.queue.get(block=True, timeout=None)
                        s_in.sendall(message.encode())
                        print(message.encode())
                    except ConnectionResetError:
                        break

        except KeyboardInterrupt:
            print("Keyboard interrupt detected")
# # parse out data
# listMsg = unityParser(data)
# if (listMsg == []):
#     continue
# strMsg = ','.join(listMsg)
#
# try:
#     msgQueue.put_nowait(json.dumps(strMsg))
# except queue.Full:
#     pass

class logSensor (threading.Thread):
   def __init__(self, threadID, name, header, serial_port, filename, queue):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.header = header
      self.serial_port = serial_port
      self.filename = filename
      self.queue = queue

   def run(self):
      print ("Starting " + self.name)
      print(" Opening port for " + self.name + ": " + self.serial_port)
      # open serial ports

      ser = serial.Serial(self.serial_port, 1000000)  # open serial port

      print(" Port status for " + self.name + ": " + str(ser.is_open))

      # add header to file
      # with open(self.filename, 'a+') as f:
      #     f.write(self.header)
      #     f.close()

      # thermopile = Thermopile(DATA_DIR, self.queue, "Thermopile")
      # sht45 = SHT4X(DATA_DIR, self.queue, "SHT45")
      # sgp = SGP(DATA_DIR, self.queue, "SGP")
      # spec = Spec(DATA_DIR, self.queue, "Spec")
      # lux = Lux(DATA_DIR, self.queue, "Lux")
      # bme = BME(DATA_DIR, self.queue, "BME")
      # blink = Blink(DATA_DIR, self.queue, "Blink")
      # # imu = IMU(DATA_DIR, self.queue, "IMU", parquet=True)
      # imu = IMU(DATA_DIR, self.queue, "IMU")
      #
      #
      # therm_pkt = 0
      # sht_pkt = 0
      # sgp_pkt = 0
      # spec_pkt = 0
      # lux_pkt = 0
      # bme_pkt = 0
      # blink_pkt = 0
      # imu_pkt = 0
      #
      # error_header_unpack = 0

      try:
          while True:
              # (1) grab serial string
              ser_string = ser.readline()
              self.queue.put(ser_string.decode("utf-8"))


      except KeyboardInterrupt:
          print("Exiting " + self.name)
          ser.close()


if __name__ == '__main__':
    start_experiment()
    # time.sleep(3);
    # log_experiment()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/