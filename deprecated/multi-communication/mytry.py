import socket
# import bluetooth
import time
from colorama import init
import serial  # 导入模块
# import zxing
import numpy as np
import time
import cv2 as cv
import os
# import pyzbar.pyzbar as pyzbar
# import RPi.GPIO as GPIO   
import threading
from uservo import UartServoManager


def init_servo():
# # 初始化串口
    # 参数配置
# 角度定义
    SERVO_PORT_NAME =  '/dev/ttyUSB1' # 舵机串口号
    # SERVO_PORT_NAME =  'COM1' # 舵机串口号
    SERVO_BAUDRATE = 115200 # 舵机的波特率
    SERVO_ID = 0  # 舵机的ID号
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
                        parity=serial.PARITY_NONE, stopbits=1,\
                        bytesize=8,timeout=0)
    # uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
    #                      parity=serial.PARITY_NONE, stopbits=1,\
    #                      bytesize=8,timeout=0)
    print(uart.is_open)
    # # 初始化舵机管理器
    global uservo
    uservo = UartServoManager(uart)
# uservo.set_servo_angle(1, 0)
#初始位置
def initail_angle():
    uservo.set_servo_angle(3,40)
    time.sleep(1)
    uservo.set_servo_angle(1,35)
    time.sleep(1)
    uservo.set_servo_angle(0,106)
    time.sleep(1)
    uservo.set_servo_angle(2,90)
    time.sleep(1)


def catch_right():
    uservo.set_servo_angle(0,106)
    time.sleep(1)
    uservo.set_servo_angle(1,-80)
    time.sleep(1)
    uservo.set_servo_angle(2,22)
    time.sleep(1)
    uservo.set_servo_angle(3,10)
    time.sleep(1)

def catch_left():
    uservo.set_servo_angle(0,77)
    time.sleep(1)
    uservo.set_servo_angle(1,-80)
    time.sleep(1)
    uservo.set_servo_angle(2,22)
    time.sleep(1)
    uservo.set_servo_angle(3,16)
    time.sleep(1)

def catch_num1():
    uservo.set_servo_angle(0,104.3)
    time.sleep(0.5)
    uservo.set_servo_angle(2,-8.1)
    time.sleep(0.5)
    uservo.set_servo_angle(1,132)
    time.sleep(2)
    uservo.set_servo_angle(3,10)
    time.sleep(1)
    uservo.set_servo_angle(1,-40)
    time.sleep(1)

def catch_num2():
    uservo.set_servo_angle(0,70.3)
    time.sleep(0.5)
    uservo.set_servo_angle(2,-23.9)
    time.sleep(0.5)
    uservo.set_servo_angle(1,141)
    time.sleep(2)
    uservo.set_servo_angle(3,10)
    time.sleep(1)
    uservo.set_servo_angle(1,-40)
    time.sleep(1)

def put_right():
    uservo.set_servo_angle(0,108.3)
    time.sleep(0.5)
    uservo.set_servo_angle(2,-10)
    time.sleep(0.5)
    uservo.set_servo_angle(1,-77)
    time.sleep(2)
    uservo.set_servo_angle(3,30)
    time.sleep(1)
    uservo.set_servo_angle(1,40)
    time.sleep(1)

def put_left():
    uservo.set_servo_angle(0,76.6)
    time.sleep(0.5)
    uservo.set_servo_angle(2,-10)
    time.sleep(0.5)
    uservo.set_servo_angle(1,-77)
    time.sleep(2)
    uservo.set_servo_angle(3,30)
    time.sleep(1)
    uservo.set_servo_angle(1,40)
    time.sleep(1)

init_servo()
initail_angle()
catch_num1()
put_left()

