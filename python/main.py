import serial
from os.path import exists
import time
import _thread as thread
import threading
from struct import *

from sensorClass import *
from thermopile import *
from sht4x import *
from sgp import *
from bme import *
from lux import *
from spec import *

gasSpec_serial = '/dev/cu.usbserial-014A1C56'

DATA_DIR = "data/"
filename_noext = 'logfile'
filename_prefix = ''
filename_extension = ".csv"
temp_header = "temp_1, temp_2, temp_3, ambient_temp, ardu_millis, epoch\n"
hr_header = "heart_rate, epoch\n"

def start_experiment():

    print("Starting experiment")

    print("Header size: " + str(headerStructSize))
    # find filename
    filename_idx = 0
    # while True:
    #     filename = DATA_DIR + filename_noext + "_" + str(filename_idx) + "_temp" + filename_extension
    #     print(filename)
    #     if not exists(filename):
    #         filename_prefix = DATA_DIR + filename_noext + "_" + str(filename_idx)
    #         break
    #     filename_idx += 1
    #
    # print("Filename prefix: " + filename_prefix)

    try:
        print("Starting threads")
        gasSpec_thread = logSensor(1, "ardu_thread", temp_header, gasSpec_serial, filename_prefix+"_temp"+filename_extension)
        # hr_thread = logSensor(2, "hr_thread", hr_header, hr_serial, filename_prefix+"_hr"+filename_extension)

        gasSpec_thread.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        print("Closing threads")

        gasSpec_thread.join()
        # hr_thread.join()

    # # grab data
    # try:
    #     while True:
    #         ardu_string = ardu_ser.readline().decode("utf-8")
    #         print(ardu_string)
    #         with open(filename, 'a+') as f:
    #             f.write(ardu_string)
    # except:
    #     ardu_ser.close()
    #     # hr_ser.close()

class logSensor (threading.Thread):
   def __init__(self, threadID, name, header, serial_port, filename):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.header = header
      self.serial_port = serial_port
      self.filename = filename
   def run(self):
      print ("Starting " + self.name)
      print(" Opening port for " + self.name + ": " + self.serial_port)
      # open serial ports
      ser = serial.Serial(self.serial_port, 115200)  # open serial port

      print(" Port status for " + self.name + ": " + str(ser.is_open))

      # add header to file
      # with open(self.filename, 'a+') as f:
      #     f.write(self.header)
      #     f.close()

      thermopile = Thermopile(DATA_DIR, "Thermopile")
      sht45 = SHT4X(DATA_DIR, "SHT45")
      sgp = SGP(DATA_DIR, "SGP")
      spec = Spec(DATA_DIR, "Spec")
      lux = Lux(DATA_DIR, "Lux")
      bme = BME(DATA_DIR, "BME")

      therm_pkt = 0
      sht_pkt = 0
      sgp_pkt = 0
      spec_pkt = 0
      lux_pkt = 0
      bme_pkt = 0

      try:
          while True:
              # ser_string = ser.readline().decode("utf-8")

              # (1) grab serial string
              ser_string = ser.readline()

              # print(headerStructSize)
              # print(ser_string[0:headerStructSize].decode("utf-8"))
              # print(bytes.fromhex(ser_string[0:headerStructSize*2].decode("utf-8")))

              # (2) unpack header
              pktType, pktID, msFromStart, epoch, payloadLen, r0, r1, r2, r3, r4 = unpack(headerStructType, bytes.fromhex(ser_string[0:headerStructSize*2].decode("utf-8")))

              # (3) check what type of packet it is and proceed accordingly
              if(THERMOPILE_PKT == pktType):
                # print("thermopile packet received: " + str(pktID))
                therm_pkt += 1
                number_of_packed_packets = int(payloadLen / thermopileStructSize)
                # thermopile.unpack_compressed_packet(ser_string, number_of_packed_packets)
              elif (SHT_PKT == pktType):
                  # print("sht packet received: " + str(pktID))
                  sht_pkt += 1
                  number_of_packed_packets = int(payloadLen / shtStructSize)
                  sht45.unpack_compressed_packet(ser_string, number_of_packed_packets)
              elif (SPEC_PKT == pktType):
                  # print("sgp packet received: " + str(pktID))
                  spec_pkt += 1
                  number_of_packed_packets = int(payloadLen / specStructSize)
                  spec.unpack_compressed_packet(ser_string, number_of_packed_packets)
              elif (BME_PKT == pktType):
                  # print("bme packet received: " + str(pktID))
                  bme_pkt += 1
                  number_of_packed_packets = int(payloadLen / bmeStructSize)
                  bme.unpack_compressed_packet(ser_string, number_of_packed_packets)
              elif (LUX_PKT == pktType):
                  # print("lux packet received: " + str(pktID))
                  lux_pkt += 1
                  number_of_packed_packets = int(payloadLen / luxStructSize)
                  lux.unpack_compressed_packet(ser_string, number_of_packed_packets)
              elif (SGP_PKT == pktType):
                  # print("lux packet received: " + str(pktID))
                  sgp_pkt += 1
                  number_of_packed_packets = int(payloadLen / sgpStructSize)
                  sgp.unpack_compressed_packet(ser_string, number_of_packed_packets)

              print("THERM: " + str(therm_pkt) + "\t" +
                    "SHT: " + str(sht_pkt) + "\t" +
                    "SGP: " + str(sgp_pkt) + "\t" +
                    "SPEC: " + str(spec_pkt) + "\t" +
                    "LUX: " + str(lux_pkt) + "\t" +
                    "BME: " + str(bme_pkt))

      except KeyboardInterrupt:
          print("Exiting " + self.name)
          ser.close()


if __name__ == '__main__':
    start_experiment()
    # time.sleep(3);
    # log_experiment()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/