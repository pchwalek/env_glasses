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

import numpy as np

from scipy.io.wavfile import write

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic


micHeaderStructType = 'HH'
micHeaderStructSize = calcsize(micHeaderStructType)

logger = logging.getLogger(__name__)

SERVICE_UUID = "0000fe80-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000fe81-0000-1000-8000-00805f9b34fb"

# SERVER_HOST = "airspecs.media.mit.edu"
SERVER_HOST = "localhost"
SERVER_PORT = 65434  # Port to listen on (non-privileged ports are > 1023)

# checkName(a, ad):
#     lambda d, ad: d.name and d.name.lower() == "AirSpec_008a65fb"
packet_idx = 0

audio_clip = np.zeros(32000)
audio_index = 0
file_index = 0

async def main(queue: asyncio.Queue):
    packet_idx = 0
    logger.info("starting scan...")
    print("starting main")
    # if args.address:
    #     device = await BleakScanner.find_device_by_address(
    #         args.address, cb=dict(use_bdaddr=args.macos_use_bdaddr)
    #     )
    #     if device is None:
    #         logger.error("could not find device with address '%s'", args.address)
    #         return
    # else:
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: ad.local_name == "AirSpec_01ad7859",
        timeout=60
    )

    while( device is None):
        # logger.error("could not find device with name '%s'", "AirSpec_008a65fb")
        print(" Bluetooth: could not find device with name '%s'", "AirSpec_008a65fb")
        print(" Bluetooth: will keep trying")
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: ad.local_name == "AirSpec_01ad7859",
            timeout=60
        )

    logger.info("connecting to device...")

    disconnected_event = asyncio.Event()

    def disconnection_handler(client):
        disconnected_event.set()

    async def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
        # global s_in
        global packet_idx, audio_index, audio_clip, file_index

        """Simple notification handler which prints the data received."""
        # logger.info("%s: %r", characteristic.description, data)

        # print(len(data))

        packet_ID, num_samples = unpack(micHeaderStructType, data[0:micHeaderStructSize])

        if(num_samples == 0):
            audio_clip /= 2147483647
            write('test_' + str(file_index) + '.wav', 48000, audio_clip)
            file_index += 1
            # print(audio_clip)
            audio_index = 0
            print("zero")
        else:
            result = unpack(str(num_samples) + 'i', data[micHeaderStructSize:(micHeaderStructSize+num_samples*4)])
            # print(str(data,"utf-16"))
            packet_idx += 1
            audio_clip[audio_index:(audio_index+num_samples)] = result
            print(result)
            audio_index += num_samples



        # print(isinstance(data, (bytes, bytearray)))
        # systemID, pktType, pktID, msFromStart, epoch, payloadLen, r0, r1, r2, r3, r4 = \
        #     unpack(headerStructType, data[0:headerStructSize])

        # try:
        # await queue.put(data)
            # s_in.sendall("test")
        # except ConnectionResetError:
        #     client.disconnect()

        # print(systemID)

    while True:
        async with BleakClient(device,disconnected_callback=disconnection_handler) as client:

            logger.info(f"Connected: {client.is_connected}")
            disconnected_event.clear()

            # svcs = await client.get_services()
            # print("Services:")
            # for service in svcs:
            #     print(service)
            # await client.start_notify(SERVICE_UUID, notification_handler)
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            try:
                while True:
                    # await asyncio.sleep(5.0)
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
                    # print("Connecting to server at: " + self.host)
                    # print(gethostbyname(self.host))
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


        # if data is None:
        #     logger.info(
        #         "Got message from client about disconnection. Exiting consumer loop..."
        #     )
        #     break
        # else:
        #     logger.info("Received callback data via async queue at %s: %r", epoch, data)

async def run_tests():
    queue = asyncio.Queue()

    main_task = main(queue)
    # consumer_task = run_queue_consumer(queue)

    await asyncio.gather(main_task)

    # await asyncio.gather(main_task, consumer_task)

if __name__ == "__main__":
    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #     s.bind((self.host, self.port))
    #     s.listen()
    #     while True:
    #         conn, addr = s.accept()
    #         print("Connection from: " + str(addr))

    # await asyncio.gather(client_task, consumer_task)



    asyncio.run(run_tests())
    # asyncio.run(main(queue))