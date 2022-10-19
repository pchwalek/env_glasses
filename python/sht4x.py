
from sensorClass import *

# define thermopile packet
shtStructType = 'ffI'
shtStructSize = calcsize(shtStructType)
shtStructLabel = ['temp','hum','timestamp']


class SHT4X(SensorClass):
    def __init__(self, filepath, queue, name="null"):
        SensorClass.__init__(self, filepath, name, queue, shtStructLabel, shtStructType)

