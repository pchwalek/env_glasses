from sensorClass import *

# define thermopile packet
blinkStructType = 'B'
blinkStructSize = calcsize(blinkStructType)
blinkStructLabel = ['sample', 'pktID','timestamp']

blink_raw_influx_header = ['signal', 'sysID', 'pktID', 'samplerate', 'diode_sat', 'timestamp']

BLINK_NEWFILE_PERIOD = 10 * 60  # in seconds

class Blink(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue=[],store_raw=0):
        SensorClass.__init__(self, filepath, name, queue, blinkStructLabel, blinkStructType)

        self.store_raw = store_raw
        self.influx_queue = influx_queue

        self.raw_influx_header = blink_raw_influx_header
    def unpack_compressed_packet(self, data, pkt_cnt, msFromStart=-1, pktID=-1, sampleRate=-1,diodeSaturated=-1,sysID=-1):
        time_recv = time.time()

        packedStructType = str(int(pkt_cnt)) + self.structType
        packedStructSize = calcsize(packedStructType)
        # unpacked_pkt = list(unpack(packedStructType,
        #               bytes.fromhex(data[(headerStructSize * 2):
        #                                 (headerStructSize * 2) + (packedStructSize * 2)].decode("utf-8"))))
        # unpacked_pkt = [str(bytes.fromhex(data[(headerStructSize * 2):
        #                                 (headerStructSize * 2) + (packedStructSize * 2)].decode("utf-8")))]
        unpacked_pkt = [data[(headerStructSize * 2):
                                               (headerStructSize * 2) + (packedStructSize * 2)]]

        newDF = pd.DataFrame(unpacked_pkt, columns=['sample'])
        newDF['sysID'] = sysID
        newDF['pktID'] = pktID
        newDF['samples'] = pkt_cnt
        newDF['samplerate'] = sampleRate
        newDF['diodeSaturated'] = diodeSaturated
        newDF['timestamp'] = msFromStart
        newDF['epoch'] = time_recv

        # print(newDF)
        if(self.influx_queue):
            data_entry = [newDF.iloc[0]['sample'],
                          newDF.iloc[0]['sysID'],
                          newDF.iloc[0]['pktID'],
                          newDF.iloc[0]['samplerate'],
                          newDF.iloc[0]['diodeSaturated'],
                          newDF.iloc[0]['timestamp']]
            self.send_to_influx(dict(zip(self.raw_influx_header, data_entry)), pkt_cnt)

        if (self.save_file_enable):
            self.append_data(newDF)

            if( (time.time() - self.last_save) > SAVE_EVERY_X_SECS):
                self.save_file()

    def append_data(self, dataEntry):
        self.df = self.df.append(dataEntry, ignore_index=True)

    def save_file(self):

        if ((self.saveIdx == 0) and (self.firstSave == 0)):
            self.firstSave = time.time()

        if (self.parquet):
            self.df.to_parquet(
                self.filepath + self.name + "_" + str(self.start_timestamp) + "_" + str(self.saveIdx) + ".parq")
        else:
            self.df.to_csv(
                self.filepath + self.name + "_" + str(self.start_timestamp) + "_" + str(self.saveIdx) + ".csv")

        if( (time.time() - self.firstSave) > BLINK_NEWFILE_PERIOD):
            self.saveIdx += 1
            self.firstSave = time.time()
            self.df = self.df[0:0] # clear dataframe to save memory

    def send_to_influx(self, pkt_dict, samples=-1):
        data = []
        if self.store_raw:
            data.append(
                "{measurement},id={id},sample_rate={sample_rate} pktID={pktID},diode_sat={diode_sat},timestamp_mcu={timestamp_mcu},signal={signal}"
                    .format(measurement="blink_raw",
                            id=pkt_dict['sysID'],
                            pktID=pkt_dict["pktID"],
                            signal="\""+str(pkt_dict["signal"])[2:-2]+"\"",
                            diode_sat=pkt_dict["diode_sat"],
                            sample_rate=pkt_dict["samplerate"],
                            timestamp_mcu=pkt_dict["timestamp"]))
        else:
            if(samples==-1):
                print(" Parsed blink samples requested but number of samples not given")
                return
            packedStructType = str(int(samples)) + self.structType
            unpacked_pkt = list(unpack(packedStructType, bytes.fromhex(pkt_dict["signal"].decode("utf-8"))))
            time_inc = 1.0/float(pkt_dict["samplerate"])
            time_offset = samples * time_inc
            for i in range(samples):
                # data.append({"measurement": "ho_feet", "tags": {"location": "coyote_creek"}, "fields": {"water_level": 1}, "time": 1})
                data.append(
                    "{measurement},id={id},sample_rate={sample_rate} pktID={pktID},diode_sat={diode_sat},timestamp_mcu={timestamp_mcu},signal={signal} {timestamp}"
                        .format(measurement="blink",
                                id=1,
                                pktID=pkt_dict["pktID"],
                                signal=unpacked_pkt[i],
                                diode_sat=pkt_dict["diode_sat"],
                                sample_rate=pkt_dict["samplerate"],
                                timestamp_mcu=pkt_dict["timestamp"]+time_inc*i,
                                timestamp=int(1000000000*(time.time()-(time_offset-time_inc*i)))))
                # data.append(
                #     "{measurement},id={id},sample_rate={sample_rate} pktID={pktID},diode_sat={diode_sat},timestamp_mcu={timestamp_mcu},signal={signal}"
                #         .format(measurement="blink",
                #                 id=1,
                #                 pktID=pkt_dict["pktID"],
                #                 signal=unpacked_pkt[i],
                #                 diode_sat=pkt_dict["diode_sat"],
                #                 sample_rate=pkt_dict["samplerate"],
                #                 timestamp_mcu=pkt_dict["timestamp"] + time_inc * i))
        self.influx_queue.put(data)
