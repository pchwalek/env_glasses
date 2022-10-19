
from sensorClass import *


# define thermopile packet
luxStructType = 'II'
luxStructSize = calcsize(luxStructType)
luxStructLabel = ['lux','timestamp']

class Lux(SensorClass):
    def __init__(self, filepath, queue, name="null"):
        SensorClass.__init__(self, filepath, name, queue, luxStructLabel, luxStructType)

