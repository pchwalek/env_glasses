
from sensorClass import *


# define thermopile packet
specStructType = 'HHHHHHHHHHHHHI'
specStructSize = calcsize(specStructType)
specStructLabel = ['_415','_445','_480','_515','_clear_1',
                   '_nir_1','_555','_590','_630','_680',
                   '_clear_2','_nir_2','flicker','timestamp']

class Spec(SensorClass):
    def __init__(self, filepath, queue, name="null", influx_queue = []):
        SensorClass.__init__(self, filepath, name, queue, specStructLabel, specStructType, influx_queue)

    def send_to_influx(self, pkt_dict):
        data = []
        id = 1
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="415",
                        id=id,
                        signal=pkt_dict["_415"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="445",
                        id=id,
                        signal=pkt_dict["_445"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="480",
                        id=id,
                        signal=pkt_dict["_480"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="515",
                        id=id,
                        signal=pkt_dict["_515"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="clear_1",
                        id=id,
                        signal=pkt_dict["_clear_1"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="nir_1",
                        id=id,
                        signal=pkt_dict["_nir_1"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="555",
                        id=id,
                        signal=pkt_dict["_555"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="590",
                        id=id,
                        signal=pkt_dict["_590"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="630",
                        id=id,
                        signal=pkt_dict["_630"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="680",
                        id=id,
                        signal=pkt_dict["_680"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="clear_2",
                        id=id,
                        signal=pkt_dict["_clear_2"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="nir_2",
                        id=id,
                        signal=pkt_dict["_nir_2"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        data.append(
            "{measurement},type={type},id={id} timestamp_mcu={timestamp_mcu},signal={signal}"
                .format(measurement="spectrometer",
                        type="flicker",
                        id=id,
                        signal=pkt_dict["flicker"],
                        timestamp_mcu=pkt_dict["timestamp"]))
        self.influx_queue.put(data)

