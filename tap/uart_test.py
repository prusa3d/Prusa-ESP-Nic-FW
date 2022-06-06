#!/bin/python

import os
import sys
from datetime import datetime
from queue import Queue
from threading import Thread, Event
from time import sleep

import serial

SERIAL = sys.argv[1] if len(sys.argv) == 2 else "/dev/ttyUSB0"
BAUD_RATE = 1500000
BUFFER_SIZE = 128

ser = serial.Serial(SERIAL, baudrate=BAUD_RATE, parity=serial.PARITY_NONE)
ser.flush()
ser.read_all()
data = Queue()
err_found = Event()


def convert_bytes(num):
    """
    this function will convert bytes to MB.... GB... etc
    """
    step_unit = 1000.0  # 1024 bad the size

    for x in ["bytes", "KB", "MB", "GB", "TB"]:
        if num < step_unit:
            return "%3.1f %s" % (num, x)
        num /= step_unit


def reader():
    start = datetime.now()
    cnt = 0
    while not err_found.is_set():
        buffer = data.get()
        read = ser.read(size=len(buffer))
        if read != buffer:
            print(f"Read  : {read.hex()}")
            print(f"Wanted: {buffer.hex()}")

            for i in range(len(buffer)):
                if buffer[i] != read[i]:
                    print(
                        f"Problem at {i}, read: {read[i]:02x}, wanted: {buffer[i]:02x}"
                    )

            err_found.set()
        cnt += len(buffer)
        now = datetime.now()
        print(
            f"Read {convert_bytes(cnt)}, {convert_bytes(cnt / (now - start).total_seconds())}/s"
        )


def writer():
    while not err_found.is_set():
        buffer = os.urandom(BUFFER_SIZE)
        data.put(buffer)
        ser.write(buffer)
        ser.flushOutput()
        sleep(0.005)


Thread(target=reader, daemon=True).start()
Thread(target=writer, daemon=True).start()

err_found.wait()
