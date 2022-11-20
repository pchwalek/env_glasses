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

thermopile_ID = {1: 'thermopile_nose_tip',
                 2: 'thermopile_nose_bridge',
                 3: 'thermopile_temple_front',
                 4: 'thermopile_temple_middle',
                 5: 'thermopile_temple_back',}

class Thermopile(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue = []):
        SensorClass.__init__(self, filepath, name, queue, thermopileStructLabel, thermopileStructType, influx_queue)

    def send_to_influx(self, pkt_dict):
        data = []
        id = 1
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement=thermopile_ID[pkt_dict["descriptor"]],
                        type="ambientRaw",
                        id=id,
                        signal=pkt_dict["ambientRaw"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement=thermopile_ID[pkt_dict["descriptor"]],
                        type="objectRaw",
                        id=id,
                        signal=pkt_dict["objectRaw"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement=thermopile_ID[pkt_dict["descriptor"]],
                        type="ambientTemp",
                        id=id,
                        signal=pkt_dict["ambientTemp"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement=thermopile_ID[pkt_dict["descriptor"]],
                        type="objectTemp",
                        id=id,
                        signal=pkt_dict["objectTemp"],
                        timestamp_mcu=pkt_dict["timestamp"]))

        self.influx_queue.put(data)