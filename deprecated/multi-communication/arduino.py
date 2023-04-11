import socket
# import bluetooth
import time
import serial  # 导入模块
# import zxing
import numpy as np
import time
import cv2 as cv
import os
import pyzbar.pyzbar as pyzbar
# import RPi.GPIO as GPIO   
# import lgpio
import pigpio
import threading
import os 

Port = os.popen('ls /dev/ttyACM*').readlines()[0][:12]
print(Port)
ser = serial.Serial(Port, 9600, timeout=0.5)
# os.popen("ls /dev/ttyACM*").readlines()
# ser.write('1231231'.encode())
time.sleep(1)
ser.write(b"0")
while True:
    try:
        # ser.write('1'.encode())

        strin = ser.readline().decode('utf-8')  # 获取arduino发送的数据
        print(strin)
        if strin[:3]=='000':
            print(strin)
        # ser.write('1'.encode())
        time.sleep(1)
    except UnicodeDecodeError:
        pass 