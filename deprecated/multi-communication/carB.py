# %%
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
    SERVO_PORT_NAME =  '/dev/ttyUSB0' # 舵机串口号
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


# %%
# parameters 
raspi_A_ipv4="192.168.2.1"
raspi_B_ipv4="192.168.2.2"
server_A_port=7892
server_B_port=7890
raspi_A_check_key="2134"
raspi_B_check_key="fds4"
rpi_A_info=(raspi_A_ipv4,server_A_port,raspi_A_check_key)
rpi_B_info=(raspi_B_ipv4,server_B_port,raspi_B_check_key)

lower_red = np.array([0,43,46])
upper_red = np.array([15,255,255])
# lower_blue
lower_blue=np.array([95, 43, 46])
upper_blue=np.array([ 124, 255, 255])

lower_g=np.array([35, 43, 46])
upper_g=np.array([ 70, 255, 255])

UPPER_BOUND=100#130#600
LOWER_BOUND=400#400#1700


# %%
# init vars 
img=0
imgPath='/home/ubuntu/multi-communication/pic/'

# %%
class Arduino_communicate():
    def __init__(self):
        self.Port = os.popen('ls /dev/ttyACM*').readlines()[0][:12]
        print('arduino',self.Port)
        self.ser = serial.Serial(self.Port, 115200, timeout=0.5)
    def write(self,data=b'0'):
        # the data should be encoded like b"0"
        time.sleep(1)
        self.ser.write(data)
    def read(self):
        try:
            # ser.write('1'.encode())
            strin = self.ser.readline().decode('utf-8')  # 获取arduino发送的数据
            print('arduino',strin)
        except UnicodeDecodeError:
            pass
        return strin
    def close(self):
        self.ser.close()

# %%
# # the client is used for car A 
# def client(server_info,send_data=""):
#     # 1. 创建tcp的套接字
#     server_ip,server_port,prefix_key=server_info
#     tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#     # 2. 链接服务器
#     server_addr = (server_ip, server_port)
#     tcp_socket.connect(server_addr)

#     # 3. 发送数据/接收数据
#     send_data=prefix_key+send_data
#     tcp_socket.send(send_data.encode("utf-8"))

#     # 4. 关闭套接字
#     tcp_socket.close()

# %%
# the server is used for car B
def server_recv(client_socket=None, ip_port=None):
    # client_socket, ip_port=socket_accpet
    # server_ip,server_port,prefix_key=server_info
    while True:
        # recv_data = client_socket.recv(1024).decode("utf-8")
        f = open(imgPath+'0.txt', 'r', encoding='gbk')
        recv_data=f.readline()
        f.close()
        # 如果接收的消息长度不为0，则将其解码输出
        if recv_data:
            # print(ip_port[0]+" said")
            # 接收客户端发送过来的请求
            # if (recv_data[:4]==prefix_key):

            exed=0
            if(len(recv_data)>1):
                print(recv_data,'\n')
            if recv_data[0:2]=="00":
                voice(recv_data[2:])
                arduino.write(b'1')
                print(recv_data,'\n')
                exed=1
            elif recv_data[0:2]=="01":
                voice(recv_data[2:])
                print(recv_data,'\n')
                catch_left()
                exed=1
                
            elif recv_data[0:2]=="02":
                voice('装配'+recv_data[2:]+'号')
                print(recv_data,'\n')
                catch_right()
                exed=1
            if(exed==1):
                f = open(imgPath+'0.txt', 'w', encoding='gbk')
                f.write('0')
                f.close()
            # os.system('scp '+imgPath+'0.txt ubuntu@'+rpi_B_info[0]+':'+imgPath)
            # client_socket.send("收到\n".encode())
        # 当客户端断开连接时，会一直发送''空字符串，所以长度为0已下线

        else:
            pass 
            # print("客户端", ip_port, "已下线\n")
            # client_socket.close()
            # break

class Server():
    def __init__(self,server_info,client_info):
        self.server_ip,self.server_port,self.prefix_key=server_info
        self.tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 2. 插入手机卡(绑定本地信息 bind)
        self.tcp_server_socket.bind(("", self.server_port))
        self.tcp_server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        # 3. 将手机设置为正常的 响铃模式(让默认的套接字由主动变为被动 listen)
        self.tcp_server_socket.listen(8)
        # 4. 等待别人的电话到来(等待客户端的链接 accept)
        self.new_client_socket, self.client_addr = self.tcp_server_socket.accept()
        if (self.client_addr[0]==client_info[0]):
            t1 = threading.Thread(target=server_recv, args=(self.new_client_socket, self.client_addr))
            # 设置线程守护
            t1.setDaemon(True)
            # 启动线程
            t1.start()
            # self.server_recv()
    # def server_recv(self):
    #     client_socket, ip_port=(self.new_client_socket,self.client_addr)
    #     # server_ip,server_port,prefix_key=server_info
    #     while True:
    #         recv_data = client_socket.recv(1024).decode("utf-8")
    #         # 如果接收的消息长度不为0，则将其解码输出
    #         if recv_data:
    #             print(ip_port[0]+" said")
    #             # 接收客户端发送过来的请求
    #             # if (recv_data[:4]==prefix_key):


    #             # print(recv_data[4:])
    #             # if recv_data[4:6]=="00":
    #             #     voice(recv_data[6:])
    #             # elif recv_data[4:6]=="01":
    #             #     voice(recv_data[6:])
    #             # elif recv_data[4:6]=="02":
    #             #     voice('正在装配'+recv_data[6:]+'号仓库')

    #             client_socket.send("收到\n".encode())
    #         # 当客户端断开连接时，会一直发送''空字符串，所以长度为0已下线
    #         else:
    #             print("客户端", ip_port, "已下线")
    #             client_socket.close()
    #             break

    # 关闭套接字
    def close(self):
        self.tcp_server_socket.close()


# %%
def voice(txt,portx="/dev/ttyUSB1",bps=9600,timex=5,):
    try:
        # #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
        # portx="/dev/ttyUSB0"
        # #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        # bps=9600
        # #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        # timex=5
        # # 打开串口，并得到串口对象
        
        # portx=os.popen('ls /dev/ttyUSB*').readlines()[0][:12]


        ser=serial.Serial(portx,bps,timeout=timex)
        # 写数据
        result=ser.write(("<G>"+txt).encode("gbk"))
        print("写总字节数:",result,'\n')
        ser.close()#关闭串口
    except Exception as e:
        print("---异常---：",e,'\n')

# %%
# def scanQRCode(srcImg):
#     # barcode = reader.decode(ImgPAth)
#     # srcImg = cv.imread(ImgPAth)
#     srcImg = cv.cvtColor(srcImg, cv.COLOR_BGR2GRAY)
#     # cv2.imshow("Image", srcImg)
#     barcodes = pyzbar.decode(srcImg,symbols=[pyzbar.ZBarSymbol.QRCODE])#QRCODE
#     # print(pyzbar.ZBarSymbol.QRCODE)
#     debarcode=0
#     qrdecode=0
#     for barcode in barcodes:
#         # barcodeData = barcode.data.encode("gbk")
#         debarcode = barcode.data.decode("utf-8")
#     # debarcode= barcodes[0].data.decode("utf-8")
#         if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
#             qrdecode='蓝色绿色'
#             break 
#         if( debarcode==r'郤｢濶ｲ扈ｿ濶ｲ'):
#             qrdecode='红色绿色'
#             break 
#         if( debarcode==r'扈ｿ濶ｲ闢晁牡'):
#             qrdecode='绿色蓝色'
#             break 
#         if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
#             qrdecode='红色蓝色'
#             break 
#     return qrdecode,debarcode


# %%
# def catch_picture(IMG_PATH=None):
#     global img
#     #调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
#     start_time=time.time()
#     cap=cv.VideoCapture(0)
#     while True:
#         #从摄像头读取图片
#         sucess,img=cap.read()
#         #转为灰度图片
#         # gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#         #显示摄像头，背景是灰度。
#         # cv2.imshow("img",gray)
#         #保持画面的持续。
#         # k=cv2.waitKey(1)
#         end_time=time.time()
#         if end_time-start_time>0.5:
#             if (IMG_PATH!=None):
#                 cv.imwrite(IMG_PATH,img)
#             # cv2.destroyAllWindows()
#             break
#     return 

# def classify_color(srcimg):
#     # frame=cv.imread(IMG_PATH)
#     frame=srcimg
#     frame=frame[UPPER_BOUND:LOWER_BOUND][:][:]
#     # 这里还是弄一个决策树或者yolov5s出来比较好
#     # 裁剪图片

#     frame=cv.resize(frame,(480,100))
#     # fram=cv2.contrast
#     # cv.imshow('Capture', frame)
#     # change to hsv model
#     hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

   
#     mask_b = cv.inRange(hsv, lower_blue, upper_blue)
#     print(mask_b.shape)
#     mask_r = cv.inRange(hsv, lower_red, upper_red)
#     mask_g = cv.inRange(hsv, lower_g,upper_g)
#     res_r = cv.bitwise_and(frame, frame, mask=mask_r)
#     res_g = cv.bitwise_and(frame, frame, mask=mask_g)
#     res_b = cv.bitwise_and(frame, frame, mask=mask_b)
#     cv.imwrite('/home/pi/Documents/gx/r.jpg',res_r)
#     cv.imwrite('/home/pi/Documents/gx/g.jpg',res_g)
#     cv.imwrite('/home/pi/Documents/gx/b.jpg',res_b)
#     kernel=np.ones((5,5),np.uint8)
#     mask_b=cv.erode(mask_b,kernel,iterations=1)
#     mask_r=cv.erode(mask_r,kernel,iterations=1)
#     mask_g=cv.erode(mask_g,kernel,iterations=1)
#     # mask_b=1-mask_b
#     # mask_r=1-mask_r
#     # mask_g=1-mask_g
#     X_axis=np.arange(0,frame.shape[1],1)
#     # mask=cv.erode(mask,kernel,iterations=1)
#     mask_r=mask_r>0
#     mask_g=mask_g>0
#     mask_b=mask_b>0
#     r_sum=(np.sum(np.nonzero(mask_r))-1)
#     g_sum=(np.sum(np.nonzero(mask_g))-1)
#     b_sum=(np.sum(np.nonzero(mask_b))-1)
#     x_r=(np.multiply(X_axis,mask_r[:]).sum()/r_sum)if r_sum>200 else -1
#     x_g=(np.multiply(X_axis,mask_g[:]).sum()/g_sum)if g_sum>200 else -1
#     x_b=(np.multiply(X_axis,mask_b[:]).sum()/b_sum)if b_sum>200 else -1
#     # res = cv.bitwise_and(frame, frame, mask=mask1)
#     # cv.imwrite('det000.jpg',res)
#     # # if __name__ == '__main__':
#     # while True:
#     #     cv.imshow('img', mask)
#     #     if cv.waitKey() == ord('q'):
#     #         break
#     # cv.destroyAllWindows()

#     return (x_r,x_g,x_b)

# %%
# if __name__=="__main__":
#     # voice("白彦小哥哥")
arduino=Arduino_communicate()
init_servo()
initail_angle()
# server=Server(rpi_B_info,rpi_A_info)
voice("启动")
server_recv()


