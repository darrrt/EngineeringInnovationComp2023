# from textwrap import indent
import serial
import struct
import os 
import time 
import serial_asyncio
import asyncio
class OutputProtocol(asyncio.Protocol):
    def __init__(self) -> None:
        super().__init__()
        self.readin=""
    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)
        # transport.serial.rts = False  # You can manipulate Serial object via transport
        transport.write('I,R123.32,T1235555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555\n'.encode('utf-8'))  # Write serial data via transport

    def data_received(self, data):
        print('data received', repr(data),data.decode("ascii"))
        # self.readin+=data.decode("utf-8")

        if b'\n' in self.readin:
            print(self.readin)
            self.readin=""
    def data_decode(self):
        pass 
    def data_write(self,data):
        self.transport.write(data)  # Write serial data via 
        print(data)

    def connection_lost(self, exc):
        print('port closed')
        self.transport.loop.stop()

    def pause_writing(self):
        print('pause writing')
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print('resume writing')

class uartSerial():
    def __init__(self,portx="/dev/ttyACM0",bps=115200) -> None:
        try:
            #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
            # portx="/dev/ttyUSB0"
            # #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
            # bps=115200
            #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
            timex=0
            # 打开串口，并得到串口对象
            self.ser=serial.Serial(portx,bps,timeout=timex)
            if self.ser.is_open():
                print("open serial",self.ser.name)
            else:
                print("serial open failed")
                exit(0)
        except:
            print("err when ini serial output")
        pass
    def send(self,id,value):
        try:
            # 写数据
            result=self.ser.write(struct.pack('>BB',id,value))
            print("写总字节数:",result,struct.pack('>BB',id,value))
        except Exception as e:
            print("---err---",e)
    def receive(self):
        data= self.ser.readlines()
        print(data)
        return data 
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


if __name__=='__main__':

    loop = asyncio.get_event_loop()
    coro = serial_asyncio.create_serial_connection(loop, OutputProtocol, '/dev/ttyACM0', baudrate=9600)
    transport, protocol = loop.run_until_complete(coro)
    
    protocol.data_write('I,R123.32,T123\n'.encode('utf-8'))

    loop.run_forever()
    # protocol.read()
    
    time.sleep(1)
    loop.close()
    # ser=uartSerial(portx=os.popen("ls /dev/ttyU*").readlines()[0][:-1])
    # # ser.send(1,18)
    # # ser.send(0,223)
    # # ser.sendBatch(2,[0,23,1,33])
    # # ser.sendBatch(2,[0,24,1,34])
    # # time.sleep(0.002)
    # # ser.sendBatch(2,[0,24,1,32])
    # # for i in range(15):
    # ser.sendBatch_V(2,[0,1.4,1,1.4])
    # time.sleep(0.002)
    # ser.close()