
from sensorClass import *


# define thermopile packet
luxStructType = 'II'
luxStructSize = calcsize(luxStructType)
luxStructLabel = ['lux','timestamp']

class Lux(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue = []):
        SensorClass.__init__(self, filepath, name, queue, luxStructLabel, luxStructType, influx_queue)

    def send_to_influx(self, pkt_dict):
        data = []
        data.append(
            "{measurement},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="lux",
                        id=1,
                        signal=pkt_dict["lux"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        self.influx_queue.put(data)

