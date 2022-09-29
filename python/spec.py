
from sensorClass import *


# define thermopile packet
specStructType = 'HHHHHHHHHHHHHI'
specStructSize = calcsize(specStructType)
specStructLabel = ['_415','_445','_480','_515','_clear_1',
                   '_nir_1','_555','_590','_630','_680',
                   '_clear_2','_nir_2','flicker','timestamp']

class Spec(SensorClass):
    def __init__(self, filepath, name="null"):
        SensorClass.__init__(self, filepath, name, specStructLabel, specStructType)

