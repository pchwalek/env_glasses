
from sensorClass import *

# define thermopile packet
sgpStructType = 'HHiiI'
sgpStructSize = calcsize(sgpStructType)
sgpStructLabel = ['srawVOC','srawNOX','voc_index_value','nox_index_value','timestamp']


class SGP(SensorClass):
    def __init__(self, filepath, name="null"):
        SensorClass.__init__(self, filepath, name, sgpStructLabel, sgpStructType)

