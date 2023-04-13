# from textwrap import indentsudo
import serial
import struct
import os 
import time 
# import serial_asyncio
# import asyncio
import binascii
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
    def __init__(self,portx="/dev/ttyACM0",bps=115200) -> None:
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
        self.send(b'I%2.2f,X%2.2f,Y%2.2f,W%2.2f,T%5.2f,O%4.2f,N%6.1f,P%2.2f!'%(self.NthInstruction,vx,vy,vw,ms,height,order,mode))
        while(self.receive().find('F')==-1):
            pass

if __name__=='__main__':

    # loop = asyncio.get_event_loop()
    # coro = serial_asyncio.create_serial_connection(loop, OutputProtocol, '/dev/ttyACM0', baudrate=115200)
    # transport, protocol = loop.run_until_complete(coro)


    # time.sleep(3)
    # protocol.data_write(b'I,R123.32,T123\n')
    # protocol.data_write(b'I,R123.32,T123\n')

    # # loop.run_forever()
    # # protocol.read()
    
    # time.sleep(1)
    # loop.close()
    # print(os.popen("ls /dev/ttyACM0").readlines()[0][:-1],'\n')
    ser=uartSerial(portx=os.popen("ls /dev/ttyACM0").readlines()[0][:-1])
    # ser.send(1,18)
    # ser.send(0,223)
    # ser.sendBatch(2,[0,23,1,33])
    # ser.sendBatch(2,[0,24,1,34])
    # time.sleep(0.002)
    # ser.sendBatch(2,[0,24,1,32])
    # for i in range(15):
    for i in range(5):
        # time.sleep(1)
        print("wait for %2d s"%(i))
        ser.receive()
    # print(ser.receive())
    print("----------------------------------")
    # ser.send(b'I%.2f,X87.3,Y76.7,W56,T6,O5,P3!'%(i))
    ser.sendInstruction(height=10)

    print("-------------start---------------------")
    # ser.sendInstruction(0.2,0.2,0,3000)
    # ser.sendInstruction(order=123321)
    ser.sendInstruction(0.2,0,0,3000)
    #ser.sendInstruction(0,0.2,0,3000)
    # ser.sendInstruction(0,0.2,0,3000)
    #     # "I,X%[^','],Y%[^','],W%[^','],T%[^','],S%[^','],E%[^','],R%[^','],V%[^','],O%[^','],P[^'\n']\n"
    #     # vx_delta,vy_delta,vw_delta,t_delta,servo0_pos,servo1_pos,servo2_pos,servo3_pos,stepper_pos,motion_type
    # time.sleep(0.1)
    # ser.receive()
    # ser.sendInstruction(0,0.2,0,1000)
    # ser.sendInstruction(0,0.2,0,1000)
    # ser.sendInstruction(0,0.2,0,1000)
    # time.sleep(0.1)
    # ser.receive()
    #     # ser.formatReadin()
    # time.sleep(4)
    # ser.receive()
    # time.sleep(6)
    # ser.receive()
    ser.close()