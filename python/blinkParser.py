import os
import pandas as pd
import blink
from struct import *
import numpy as np

DATA_DIR = "data/"
filename_prefix = ''
filename_suffix = '_parsed'
filename_extension = ".csv"

blinkStructType = 'B'

blinkParsedHeader = ['sample', 'msFromStart', 'epoch']

def parseBlinkFile(inFile, outFile):
    df = pd.read_csv(DATA_DIR + inFile)
    df_parsed = pd.DataFrame(columns=blinkParsedHeader)


    for index, row in df.iterrows():
        pkt_cnt = row['samples']
        sampleRate = row['samplerate']
        lastMsFromStart = row['timestamp']
        lastEpoch = row['epoch']
        packedStructType = str(int(pkt_cnt)) + blinkStructType
        # packedStructSize = calcsize(packedStructType)

        data = row['sample'][2:-1].encode('utf-8').decode('unicode_escape').encode("raw_unicode_escape")
        unpacked_pkt = list(unpack(packedStructType,
                                   data))
        tempdf = pd.DataFrame(unpacked_pkt, columns=['sample'])
        tempdf['msFromStart'] = np.arange(lastMsFromStart - (1000*pkt_cnt/sampleRate),lastMsFromStart, 1000.0/sampleRate)
        tempdf['epoch'] = lastEpoch
        df_parsed = df_parsed.append(tempdf, ignore_index=True)

    df_parsed.to_csv(outFile)


for file in os.listdir(DATA_DIR):
    if file.startswith("Blink"):
        print(file)
        outFile = os.path.splitext(file)[0] + filename_suffix + filename_extension
        if not os.path.exists(DATA_DIR+outFile):
            parseBlinkFile(file, DATA_DIR + outFile)

