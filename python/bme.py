
from sensorClass import *


# define thermopile packet
bmeStructType = 'qfBBBB'
bmeStructSize = calcsize(bmeStructType)
bmeStructLabel = ['timestamp_bme','signal','signal_dimensions','sensor_id','accuracy','nOutputs']


class SGP(SensorClass):
    def __init__(self, filepath, name="null"):
        SensorClass.__init__(self, filepath, name, bmeStructLabel, bmeStructType)

