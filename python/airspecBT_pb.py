# -*- coding: utf-8 -*-
"""
Notifications
-------------
Example showing how to add notifications to a characteristic and handle the responses.
Updated on 2019-07-03 by hbldh <henrik.blidh@gmail.com>
"""

import argparse
import asyncio
import logging

import binascii
from struct import *
from sensorClass import *

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

import numpy as np
import message_pb2 as MessagePb

sensorPacket = MessagePb.SensorPacket()

logger = logging.getLogger(__name__)

# AIRSPEC_UID = "AirSpec_01ad7510",
# AIRSPEC_UID = "AirSpec_01ad6cff",
# AIRSPEC_UID = "AirSpec_01ad6f6b", # glasses 1
# AIRSPEC_UID = "AirSpec_01ad7855", # glasses 2
# AIRSPEC_UID = "AirSpec_01ad71de", # glasses 3
# AIRSPEC_UID = "AirSpec_01ad7052",  # glasses 4
# AIRSPEC_UID = "AirSpec_01ad72c2", # glasses 5
# AIRSPEC_UID = "AirSpec_01ad7040", # glasses 6
# AIRSPEC_UID = "AirSpec_01ad6d72", # glasses 7
# AIRSPEC_UID = "AirSpec_01ad6ce3", # glasses 9
# AIRSPEC_UID = "AirSpec_01ad6e53", # glasses 10
# AIRSPEC_UID = "AirSpec_01ad743c", # glasses 11
# AIRSPEC_UID = "AirSpec_01ad7677", # glasses 12
# AIRSPEC_UID = "AirSpec_01ad6e65", # glasses 13
# AIRSPEC_UID = "AirSpec_01ad7ae6", # glasses 14
AIRSPEC_UID = "AirSpec_01ad6fa1" # glasses 15
# AIRSPEC_UID = "AirSpec_01ad7ae6",  # glasses 16
# AIRSPEC_UID = "AirSpec_01ad71bf", # glasses 17
# AIRSPEC_UID = "AirSpec_01ad7859", # glasses 18

SERVICE_UUID = "0000fe80-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000fe81-0000-1000-8000-00805f9b34fb"

# SERVER_HOST = "airspecs.media.mit.edu"
SERVER_HOST = "localhost"
SERVER_PORT = 65434  # Port to listen on (non-privileged ports are > 1023)
SEND_TO_SERVER = False

UNKNOWN_PACKET_TYPE = 0
# PPG_RED = 1
# PPG_IR = 2
SPECTROMETER = 3
BME = 4
CO2 = 5
IMU = 6
THERMOPILE = 7
LUX = 8
LIDAR = 9
MIC = 10
SHT = 11
SGP = 12
BLINK = 13
MIC_LVL = 14

sensorPacketTracker = np.zeros(14)

def sensorPrintHelperFunc(packet, show_payload=False):
    # sensorPacketTracker[pktIdxRx] += 1

    # print(packet.WhichOneof("payload"))
    if(packet.WhichOneof("payload") == "lux_packet"):
        sensorPacketTracker[LUX] += 1
        if show_payload:
            print(packet.lux_packet)
    elif(packet.WhichOneof("payload") == "sgp_packet"):
        sensorPacketTracker[SGP] += 1
        # if show_payload:
        print(packet.sgp_packet)
    elif (packet.WhichOneof("payload") == "bme_packet"):
        sensorPacketTracker[BME] += 1
        if show_payload:
            print(packet.bme_packet)
    elif (packet.WhichOneof("payload") == "blink_packet"):
        sensorPacketTracker[BLINK] += 1
        # print(packet.blink_packet)
        if show_payload:
            print(packet.blink_packet)
    elif (packet.WhichOneof("payload") == "sht_packet"):
        sensorPacketTracker[SHT] += 1
        print(packet.sht_packet)
        if show_payload:
            print(packet.sht_packet)
    elif (packet.WhichOneof("payload") == "spec_packet"):
        sensorPacketTracker[SPECTROMETER] += 1
        if show_payload:
            print(packet.spec_packet)
    elif (packet.WhichOneof("payload") == "therm_packet"):
        sensorPacketTracker[THERMOPILE] += 1
        # print(packet.therm_packet)
        # if show_payload:
        print(packet.therm_packet)
    elif (packet.WhichOneof("payload") == "imu_packet"):
        sensorPacketTracker[IMU] += 1
        if show_payload:
            print(packet.imu_packet)
    elif (packet.WhichOneof("payload") == "mic_packet"):
        sensorPacketTracker[MIC] += 1
        if show_payload:
            print(packet.mic_packet)
    elif (packet.WhichOneof("payload") == "mic_level_packet"):
        sensorPacketTracker[MIC_LVL] += 1
        if show_payload:
            print(packet.mic_level_packet)


    print("UNK: " + str(sensorPacketTracker[UNKNOWN_PACKET_TYPE]) , end ="\t")
    print("SPEC: " + str(sensorPacketTracker[SPECTROMETER]), end="\t")
    print("BME: " + str(sensorPacketTracker[BME]), end="\t")
    print("IMU: " + str(sensorPacketTracker[IMU]), end="\t")
    print("THERM: " + str(sensorPacketTracker[THERMOPILE]), end="\t")
    print("LUX: " + str(sensorPacketTracker[LUX]), end="\t")
    print("MIC: " + str(sensorPacketTracker[MIC]), end="\t")
    print("MIC_LVL: " + str(sensorPacketTracker[MIC_LVL]), end="\t")
    print("SHT: " + str(sensorPacketTracker[SHT]), end="\t")
    print("SGP: " + str(sensorPacketTracker[SGP]), end="\t")
    print("Blink: " + str(sensorPacketTracker[BLINK]))

async def main(queue: asyncio.Queue):
    global sensorPacketTracker

    sensorPacketTracker = np.zeros(15)

    logger.info("starting scan...")
    print("starting main")

    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: ad.local_name == AIRSPEC_UID,  # glasses 4
        # lambda d, ad: ad.local_name == "AirSpec_01ad6fa1",
        timeout=60
    )

    while( device is None):
        # logger.error("could not find device with name '%s'", "AirSpec_008a65fb")
        print(" Bluetooth: could not find device with name '%s'", "AirSpec_008a65fb")
        print(" Bluetooth: will keep trying")
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: ad.local_name == "AirSpec_01AD7510",
            timeout=60
        )

    logger.info("connecting to device...")

    disconnected_event = asyncio.Event()

    def disconnection_handler(client):
        disconnected_event.set()

    async def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):

        sensorPacket.ParseFromString(bytes(data))

        if(SEND_TO_SERVER):
            await queue.put(data)

        sensorPrintHelperFunc(sensorPacket, False)

    while True:
        async with BleakClient(device,disconnected_callback=disconnection_handler) as client:

            logger.info(f"Connected: {client.is_connected}")
            disconnected_event.clear()

            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            try:
                while True:
                    await disconnected_event.wait()
                    if(not client.is_connected):
                        print(" Bluetooth: connection lost. Attempting to reconnect.")
                        await client.disconnect()
                        await asyncio.sleep(5.0)
                        break

            except:
                print(" Bluetooth: Exception received so stopping notification subscription")
                await client.stop_notify(CHARACTERISTIC_UUID)
                break

async def run_queue_consumer(queue: asyncio.Queue):
    logger.info("Starting queue consumer")

    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_in:
            while True:
                try:
                    s_in.connect((SERVER_HOST, SERVER_PORT))
                except OSError:
                    print("Can't connect to server! Generated OSError. Retrying in: ", end=" ")
                    time.sleep(1)
                    print("5.. ", end=" ")
                    time.sleep(1)
                    print("4.. ", end=" ")
                    time.sleep(1)
                    print("3.. ", end=" ")
                    time.sleep(1)
                    print("2.. ", end=" ")
                    time.sleep(1)
                    print("1.. ")
                    time.sleep(1)
                    continue
                except ConnectionRefusedError:
                    print("ERROR: can't connect to python script")
                    continue
                print("connected to server")
                break


            while True:
                # Use await asyncio.wait_for(queue.get(), timeout=1.0) if you want a timeout for getting data.
                data = await queue.get()
                if data is None:
                    break
                try:
                    s_in.sendall(binascii.hexlify(data))
                except (ConnectionAbortedError, ConnectionResetError):
                    print("conn aborted")
                    break

async def run_experiments():
    queue = asyncio.Queue()

    main_task = main(queue)
    if(SEND_TO_SERVER):
        consumer_task = run_queue_consumer(queue)

    if(SEND_TO_SERVER):
        await asyncio.gather(main_task, consumer_task)
    else:
        await asyncio.gather(main_task)

if __name__ == "__main__":
    asyncio.run(run_experiments())
