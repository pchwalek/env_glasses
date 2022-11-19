
from sensorClass import *

# define thermopile packet
shtStructType = 'ffI'
shtStructSize = calcsize(shtStructType)
shtStructLabel = ['temp','hum','timestamp']


class SHT4X(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue = []):
        SensorClass.__init__(self, filepath, name, queue, shtStructLabel, shtStructType, influx_queue)

    def send_to_influx(self, pkt_dict):
        data = []
        id = 1
        data.append(
            "{measurement},type={type},id={id},timestamp_mcu={timestamp_mcu} signal={signal}"
                .format(measurement="sht45",
                        type="temperature",
                        id=id,
                        signal=pkt_dict["temp"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id},timestamp_mcu={timestamp_mcu} signal={signal}"
                .format(measurement="sht45",
                        type="humidity",
                        id=id,
                        signal=pkt_dict["hum"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        self.influx_queue.put(data)

