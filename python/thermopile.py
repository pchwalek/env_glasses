from struct import *
import pandas as pd
import time

from sensorClass import *

# define thermopile packet
thermopileStructType = 'BIHIff'
thermopileStructSize = calcsize(thermopileStructType)
thermopileStructLabel = ['descriptor', 'timestamp', 'ambientRaw', 'objectRaw', 'ambientTemp', 'objectTemp']
THERMOPLE_NOSE_TIP_ID		        = 1
THERMOPLE_NOSE_BRIDGE_ID			= 2
THERMOPLE_TEMPLE_FRONT_ADDR_ID		= 3
THERMOPLE_TEMPLE_MID_ADDR_ID		= 4
THERMOPLE_TEMPLE_BACK_ADDR_ID		= 5

class Thermopile(SensorClass):
    def __init__(self, filepath, name="null"):
        SensorClass.__init__(self, filepath, name, thermopileStructLabel, thermopileStructType)

