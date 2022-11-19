from sensorClass import *

# define thermopile packet
blinkStructType = 'B'
blinkStructSize = calcsize(blinkStructType)
blinkStructLabel = ['sample', 'pktID','timestamp']

BLINK_NEWFILE_PERIOD = 10 * 60  # in seconds

class Blink(SensorClass):
    def __init__(self, filepath, queue, name="null"):
        SensorClass.__init__(self, filepath, name, queue, blinkStructLabel, blinkStructType)

    def unpack_compressed_packet(self, data, pkt_cnt, msFromStart=-1, pktID=-1, sampleRate=-1,diodeSaturated=-1):
        time_recv = time.time()

        packedStructType = str(int(pkt_cnt)) + self.structType
        packedStructSize = calcsize(packedStructType)
        # unpacked_pkt = list(unpack(packedStructType,
        #               bytes.fromhex(data[(headerStructSize * 2):
        #                                 (headerStructSize * 2) + (packedStructSize * 2)].decode("utf-8"))))
        unpacked_pkt = [str(bytes.fromhex(data[(headerStructSize * 2):
                                        (headerStructSize * 2) + (packedStructSize * 2)].decode("utf-8")))]
        newDF = pd.DataFrame(unpacked_pkt, columns=['sample'])
        newDF['pktID'] = pktID
        newDF['samples'] = pkt_cnt
        newDF['samplerate'] = sampleRate
        newDF['diodeSaturated'] = diodeSaturated
        newDF['timestamp'] = msFromStart
        newDF['epoch'] = time_recv
        self.append_data(newDF)
        # self.save_file()

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
