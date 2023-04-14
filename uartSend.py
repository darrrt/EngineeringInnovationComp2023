#! /usr/bin/python3
# from textwrap import indentsudo
import serial
import struct
import os 
import time 
# import serial_asyncio
# import asyncio
import binascii
from servo.Servo import *
# from servo.Adafruit_PCA9685 import *

SERVO0_0m=262
SERVO0_0l=235
SERVO0_0r=290
SERVO0_1=70
SERVO0_2=20
SERVO0_3=118
SERVO1_0=5
SERVO1_1=300
SERVO1_2=150
SERVO2_0=260
SERVO2_1=90
SERVO2_2=180
SERVO2_3=0
SERVO3_0=195
SERVO3_1=220
SERVO3_2=120
SERVO0=4
SERVO1=1
SERVO2=2
SERVO3=3

SERVO0_SET=[0,0,0]
SERVO0_SET[0]=SERVO0_0l
SERVO0_SET[1]=SERVO0_0m
SERVO0_SET[2]=SERVO0_0r

SET_1=[0,0,0]
SET_1[0]=SERVO0_2
SET_1[1]=SERVO0_1
SET_1[2]=SERVO0_3


# class OutputProtocol(asyncio.Protocol):
#     def __init__(self) -> None:
#         super().__init__()
#         # self.readin=""
#         self.transport = None
#     def connection_made(self, transport):
#         self.transport = transport
#         self.buf = bytes()
#         print('port opened', transport)
#         transport.serial.rts = False  # You can manipulate Serial object via transport
#         # transport.write(b'I,R123.32,T1235555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555\n')  # Write serial data via transport

#     # def data_received(self, data):
#     #     print('data received', repr(data),data.decode("ascii"))
#     #     # self.readin+=data.decode("utf-8")

#     #     if b'\n' in self.readin:
#     #         print(self.readin)
#     #         self.readin=""
#     def data_received(self, data):
#         """Store characters until a newline is received.
#         """
#         self.buf += data
#         print(f'Reader received: {data.decode()}')
#         if b'\n' in self.buf:
#             lines = self.buf.split(b'\n')
#             self.buf = lines[-1]  # whatever was left over
#             for line in lines[:-1]:
#                 print(f'Reader received: {line.decode()}')
#                 # self.msgs_recvd += 1
#         # if self.msgs_recvd == 4:
#         #     self.transport.close()
#     def data_decode(self):
#         pass 
#     def data_write(self,data):
#         self.transport.write(data)  # Write serial data via 
#         print(data)

#     def connection_lost(self, exc):
#         print('port closed')
#         self.transport.loop.stop()

#     def pause_writing(self):
#         print('pause writing')
#         print(self.transport.get_write_buffer_size())

#     def resume_writing(self):
#         print(self.transport.get_write_buffer_size())
#         print('resume writing')

class uartSerial():
    def __init__(self,portx="/dev/ttyACM1",bps=115200) -> None:
        self.buf = bytes()
        try:
            #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
            # portx="/dev/ttyUSB0"
            # #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
            # bps=115200
            #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
            timex=1
            # if not add this time, the pi will not contain the message doesn't be accepted by the arduino
            # 打开串口，并得到串口对象
            self.ser=serial.Serial(portx,bps,timeout=timex)
            if self.ser.is_open:
                print("open serial",self.ser.name)
            else:
                print("serial open failed")
                exit(0)
        except:
            print("err when ini serial output")

        self.currentReadinData=""
        self.serialIn=""
        self.NthInstruction=0
        pass
    def send(self,value):
        try:
            # 写数据
            result=self.ser.write(value)
            # result=self.ser.write(struct.pack('>BB',id,value))
            print("写总字节数:",result,"写入内容",value)
        except Exception as e:
            print("---err---",e)
    def receive(self):
        # data= self.ser.readlines()
        indata=""
        indata= self.ser.readline()
        print(indata)
        # if indata!=[]: 
        #     indata=indata[0] 
        # else:
        #     indata=""
        if indata!="":
            self.currentReadinData= indata[:-1].decode()
            print('in waiting',self.ser.in_waiting)
            indata=self.ser.read(self.ser.in_waiting)  
            self.currentReadinData+=indata[:-1].decode()
            print('readin',self.currentReadinData)
            # print(self.currentReadinData)
            return self.currentReadinData
        else:
            print("No readin from serial")
            raise ""
        # self.buf += data
        # print(f'Reader received: {data.decode()}')
        # if b'\n' in self.buf:
        #     lines = self.buf.split(b'\n')
        #     self.buf = lines[-1]  # whatever was left over
        #     for line in lines[:-1]:
        #         print(f'Reader received: {line.decode()}')
        # return data 
    def formatReadin(self):
        self.serialInfos=[]
        self.receive()
        print(self.currentReadinData)
        while(self.currentReadinData.find('!')==-1):
            self.receive()
            self.serialIn+=self.currentReadinData
        lastidx=0
        idx=self.serialIn.find('!')
        print("idx",idx)
        while(idx!=-1):
            self.serialInfos+=[self.serialIn[lastidx:idx+1]]
            lastidx=idx 
        self.serialIn=self.serialIn[idx:]
        print(self.serialInfos)
    # def sendBatch(self,id_num,values):
    #     try:
    #         # 写数据
    #         for i in range(id_num):
    #             result=self.ser.write(struct.pack('>BB',values[i*2],values[i*2+1]))
    #             print("写总字节数:",result,struct.pack('>BB',values[i*2],values[i*2+1]))
    #     except Exception as e:
    #         print("---err---",e)
    # def sendBatch_V(self,id_num,values):
    #     try:
    #         # 写数据
    #         for i in range(id_num):
    #             result=self.ser.write(struct.pack('>BB',int(values[i*2]),int(values[i*2+1]/3.3*255)))
    #             print("写总字节数:",result,struct.pack('>BB',int(values[i*2]),int(values[i*2+1]/3.3*255)),int(values[i*2+1]/3.3*255))
    #     except Exception as e:
    #         print("---err---",e)
    def close(self):
        self.ser.close()#关闭串口

    def sendInstruction(self,vx=-999,vy=-999,vw=-999,ms=-999,height=-999,mode=-999,order=-999):
        self.NthInstruction+=1
        self.send(b'I%2.1f,X%2.1f,Y%2.1f,W%2.1f,T%2.1f,O%2.1f,N%2.1f,P%2.1f!'%(self.NthInstruction,vx,vy,vw,ms,height,order,mode))
        while(self.receive().find('F')==-1):
            pass
        self.receive()

if __name__=='__main__':
    ServoControl(SERVO0_0l,SERVO0)
    ServoControl(SERVO1_0,SERVO1)
    ServoControl(SERVO2_0,SERVO2)
    ServoControl(SERVO3_0,SERVO3)
    ser=uartSerial(portx=os.popen("ls /dev/ttyACM*").readlines()[0][:-1])

    for i in range(5):
        # time.sleep(1)
        print("wait for %2d s"%(i))
        ser.receive()
    # print(ser.receive())
    print("----------------------------------")
    ser.sendInstruction(height=10)

    print("-------------start---------------------")
    # ser.sendInstruction(0.2,0.2,0,3000)
    
    # ser.sendInstruction(height=-10)
    #ser.sendInstruction(order=123321)
    # ser.sendInstruction(0,0,3.14,5000)
    # ser.sendInstruction(0,0.2,0,1800)
    # ser.sendInstruction(0.2,0,0,1800)
    ser.sendInstruction(height=10)
    # QRposTO(SERVO0_0l,0)
    # catch_picture("0.jpg")
    # squ=QRScan("0.jpg")
    # print(squ)

    ser.close()