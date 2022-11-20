from sensorClass import *

# define thermopile packet
bmeStructType = 'qfBBBx'
bmeStructSize = calcsize(bmeStructType)
bmeStructLabel = ['timestamp_ns', 'signal', 'signal_dimensions', 'sensor_id', 'accuracy']

# note: for sensor ID's, reference bsec_virtual_sensor_t enum here:
#       https://github.com/BoschSensortec/BSEC-Arduino-library/blob/master/src/inc/bsec_datatypes.h
BSEC_IAQ = 1
BSEC_STATIC_IAQ = 2
BSEC_CO2_EQ = 3
BSEC_BREATH_VOC_EQ = 4
BSEC_RAW_TEMP = 6
BSEC_RAW_PRESSURE = 7
BSEC_RAW_HUM = 8
BSEC_RAW_GAS = 9
BSEC_OUTPUT_STABLE = 12
BSEC_OUTPUT_RUN_STATUS = 13
BSEC_HEAT_COMP_TEMP = 14
BSEC_HEAT_COMP_HUM = 15
BSEC_OUT_COMP_GAS = 18
BSEC_OUT_GAS_PERC = 21

# accuracy as defined by BSEC library
HIGH_ACCURACY = 3
MED_ACCURACY = 2
LOW_ACCURACY = 1
NOT_ACCURATE = 0

class BME(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue = [], system_id=1):
        SensorClass.__init__(self, filepath, name, queue, bmeStructLabel, bmeStructType, influx_queue)
        self.system_id = system_id

    def send_to_influx(self, pkt_dict):
        data = []
        data.append(
            "{measurement},sensor_id={sensor_id},id={id} accuracy={accuracy},timestamp_ns={timestamp_ns},signal={signal}"
                .format(measurement="bme",
                        sensor_id=pkt_dict["sensor_id"],
                        id=1,
                        signal=pkt_dict["signal"],
                        accuracy=pkt_dict["accuracy"],
                        timestamp_ns=pkt_dict["timestamp_ns"]))
        self.influx_queue.put(data)

