from struct import *
import pandas as pd
import time
import struct as struct
import ctypes

from sensorClass import *

# define thermopile packet
imuStructType = 'BBBBBBBBBBBB'
imuStructSize = calcsize(imuStructType)
imuStructLabel = ['ACCEL_XOUT_H', 'ACCEL_XOUT_L', 'ACCEL_YOUT_H', 'ACCEL_YOUT_L', 'ACCEL_ZOUT_H', 'ACCEL_ZOUT_L',
                  'GYRO_ZOUT_H', 'GYRO_ZOUT_L', 'GYRO_YOUT_H', 'GYRO_YOUT_L', 'GYRO_XOUT_H', 'GYRO_XOUT_L']
imuParsedStructLabel = ['ACCEL_XOUT', 'ACCEL_YOUT', 'ACCEL_ZOUT',
                        'GYRO_ZOUT', 'GYRO_YOUT', 'GYRO_XOUT']
imu_raw_influx_header = ['signal', 'sysID', 'pktID', 'timestamp']

IMU_RESOLUTION = 2048  # lsb/g

IMU_NEWFILE_PERIOD = 10 * 60  # in seconds


class IMU(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue=[], parquet=False, store_raw=0):
        SensorClass.__init__(self, filepath, name, queue, imuStructLabel, imuStructType)

        self.header = imuStructLabel
        self.header.append("pktID")
        self.header.append("msFromStart")
        self.header.append("epoch")

        self.parsedHeader = imuParsedStructLabel
        self.parsedHeader.append("pktID")
        self.parsedHeader.append("msFromStart")
        self.parsedHeader.append("epoch")
        self.df = pd.DataFrame(columns=self.parsedHeader)

        self.influx_queue = influx_queue

        self.parquet = parquet
        self.store_raw = store_raw

        self.raw_influx_header = imu_raw_influx_header

    def unpack_compressed_packet(self, data, pkt_cnt, msFromStart=-1, pktID=-1, sysID=-1):
        time_recv = time.time()
        # print(len(data))
        # print("pkt_cnt: " + str(pkt_cnt))
        if self.store_raw == 0:
            for idx in range(pkt_cnt):
                try:
                    # print(data[(headerStructSize * 2) + idx * (self.structSize * 2):
                    #            (headerStructSize * 2) + (idx + 1) * (
                    #                    self.structSize * 2)].decode("utf-8"))
                    # print(bytes.fromhex(data[(headerStructSize * 2) + idx * (self.structSize * 2):
                    #                          (headerStructSize * 2) + (idx + 1) * (
                    #                                  self.structSize * 2)].decode("utf-8")))
                    # print((headerStructSize * 2) + idx * (self.structSize * 2))
                    # print((headerStructSize * 2) + (idx + 1) * (self.structSize * 2))

                    unpacked_pkt = list(unpack(self.structType,
                                               bytes.fromhex(data[(headerStructSize * 2) + idx * (self.structSize * 2):
                                                                  (headerStructSize * 2) + (idx + 1) * (
                                                                          self.structSize * 2)].decode("utf-8"))))
                    # print(unpacked_pkt)
                    parsed_pkt = []
                    parsed_pkt.append(ctypes.c_int16((unpacked_pkt[0] << 8) | unpacked_pkt[1]).value / 2048)  # unit: g
                    parsed_pkt.append(ctypes.c_int16((unpacked_pkt[2] << 8) | unpacked_pkt[3]).value / 2048)  # unit: g
                    parsed_pkt.append(ctypes.c_int16((unpacked_pkt[4] << 8) | unpacked_pkt[5]).value / 2048)  # unit: g
                    parsed_pkt.append(ctypes.c_int16(
                        (unpacked_pkt[6] << 8) | unpacked_pkt[7]).value / 16.4)  # unit: degrees-per-second (dps)
                    parsed_pkt.append(ctypes.c_int16(
                        (unpacked_pkt[8] << 8) | unpacked_pkt[9]).value / 16.4)  # unit: degrees-per-second (dps)
                    parsed_pkt.append(ctypes.c_int16(
                        (unpacked_pkt[10] << 8) | unpacked_pkt[11]).value / 16.4)  # unit: degrees-per-second (dps)
                except ValueError:
                    print("Value Error in IMU Thread")
                    return
                except struct.error:
                    print("Struct Error in IMU Thread")
                    return
                parsed_pkt.append(pktID)
                parsed_pkt.append(msFromStart)
                parsed_pkt.append(time_recv)
                self.append_data(parsed_pkt)
        else:
            self.append_data([data.decode("utf-8")])

        if(self.influx_queue):
            if(self.store_raw):
                data_entry = [data.decode("utf-8"), sysID, pktID, msFromStart]
                self.send_to_influx(dict(zip(self.raw_influx_header, data_entry)))

        if (self.save_file_enable):
            if ((time.time() - self.last_save) > SAVE_EVERY_X_SECS):
                self.save_file()
                self.last_save = time.time()

    def save_file(self):

        if ((self.saveIdx == 0) and (self.firstSave == 0)):
            self.firstSave = time.time()

        if (self.parquet):
            self.df.to_parquet(
                self.filepath + self.name + "_" + str(self.start_timestamp) + "_" + str(self.saveIdx) + ".parq")
        else:
            self.df.to_csv(
                self.filepath + self.name + "_" + str(self.start_timestamp) + "_" + str(self.saveIdx) + ".csv")

        if ((time.time() - self.firstSave) > IMU_NEWFILE_PERIOD):
            self.saveIdx += 1
            self.firstSave = time.time()
            self.df = self.df[0:0]  # clear dataframe to save memory

    def send_to_influx(self, pkt_dict):
        data = []
        data.append(
            "{measurement},id={id} pktID={pktID},timestamp_mcu={timestamp_mcu},signal=\"{signal}\""
                .format(measurement="imu_raw",
                        id=pkt_dict["sysID"],
                        pktID=pkt_dict["pktID"],
                        signal=pkt_dict["signal"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        self.influx_queue.put(data)