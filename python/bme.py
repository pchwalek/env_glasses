from sensorClass import *

# define thermopile packet
bmeStructType = 'qfBBBx'
bmeStructSize = calcsize(bmeStructType)
bmeStructLabel = ['timestamp_ns','signal','signal_dimensions','sensor_id','accuracy']

# note: for sensor ID's, reference bsec_virtual_sensor_t enum here:
#       https://github.com/BoschSensortec/BSEC-Arduino-library/blob/master/src/inc/bsec_datatypes.h

class BME(SensorClass):
    def __init__(self, filepath, name="null"):
        SensorClass.__init__(self, filepath, name, bmeStructLabel, bmeStructType)

