
from sensorClass import *

# define thermopile packet
sgpStructType = 'HHiiI'
sgpStructSize = calcsize(sgpStructType)
sgpStructLabel = ['srawVOC','srawNOX','voc_index_value','nox_index_value','timestamp']


class SGP(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue = []):
        SensorClass.__init__(self, filepath, name, queue, sgpStructLabel, sgpStructType, influx_queue)

    def send_to_influx(self, pkt_dict):
        data = []
        id = 1
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="sgp",
                        type="srawVOC",
                        id=id,
                        signal=pkt_dict["srawVOC"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="sgp",
                        type="srawNOX",
                        id=id,
                        signal=pkt_dict["srawNOX"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="sgp",
                        type="voc_index_value",
                        id=id,
                        signal=pkt_dict["voc_index_value"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="sgp",
                        type="nox_index_value",
                        id=id,
                        signal=pkt_dict["nox_index_value"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        self.influx_queue.put(data)

