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
    while True:
        read = ser.readline()
        print(read.decode())


def writer():
    cnt = 0
    while True:
        ser.write(cnt.to_bytes(4, 'little', signed=False))
        cnt += 1


Thread(target=reader, daemon=True).start()
Thread(target=writer, daemon=True).start()

err_found.wait()
