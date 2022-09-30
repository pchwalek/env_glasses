
from sensorClass import *


# define thermopile packet
bmeStructType = 'qfBBBx'
bmeStructSize = calcsize(bmeStructType)
bmeStructLabel = ['timestamp_ns','signal','signal_dimensions','sensor_id','accuracy']

class BME(SensorClass):
    def __init__(self, filepath, name="null"):
        SensorClass.__init__(self, filepath, name, bmeStructLabel, bmeStructType)

