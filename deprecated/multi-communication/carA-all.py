# %%
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
#--------舵机-----------
from uservo import UartServoManager
import uservo 

def catch_num_one():
    uservo.set_servo_angle(2, 40)

    #uservo.set_servo_angle(3, -40)
    time.sleep(1)

    uservo.set_servo_angle(0, 45.4) 
    time.sleep(1)
    uservo.set_servo_angle(1, 37.3)

    time.sleep(1)
    #uservo.set_servo_angle(2, 27.7)
    uservo.set_servo_angle(3, 17) 
    time.sleep(1)
    uservo.set_servo_angle(5,-10)
    time.sleep(2)
    uservo.set_servo_angle(1,0)
    time.sleep(1)

def catch_num_two():
    uservo.set_servo_angle(2, 40)

    #uservo.set_servo_angle(3, -40)
    time.sleep(0.5)

    uservo.set_servo_angle(0, -3.1) 
    time.sleep(0.5)
    uservo.set_servo_angle(1, 37.3)

    time.sleep(0.5)
    #uservo.set_servo_angle(2, 27.7)
    uservo.set_servo_angle(3, 17) 
    time.sleep(0.5)
    
    uservo.set_servo_angle(5,-10)
    time.sleep(2)
    uservo.set_servo_angle(1,0)

def catch_num_three():
    uservo.set_servo_angle(2, 40)

    #uservo.set_servo_angle(3, -40)
    time.sleep(0.5)

    uservo.set_servo_angle(0, -26) 
    time.sleep(0.5)
    uservo.set_servo_angle(1, 38.3)

    time.sleep(0.5)
    #uservo.set_servo_angle(2, 27.7)
    uservo.set_servo_angle(3, 22) 
    time.sleep(0.5)
    
    uservo.set_servo_angle(5,-10)
    time.sleep(2)
    uservo.set_servo_angle(1,0)

def catch_num_four():
    uservo.set_servo_angle(2, 10)

    #uservo.set_servo_angle(3, -40)
    time.sleep(0.5)

    uservo.set_servo_angle(0, -92) 
    time.sleep(0.5)

    uservo.set_servo_angle(3, 0) 
    time.sleep(0.5)

    uservo.set_servo_angle(1, 53.3)

    time.sleep(0.5)
    uservo.set_servo_angle(5,-10)
    time.sleep(2)
    uservo.set_servo_angle(1,0)
    time.sleep(0.5)

def catch_num_five():
    uservo.set_servo_angle(2, 34)

    #uservo.set_servo_angle(3, -40)
    time.sleep(0.5)

    uservo.set_servo_angle(0, -54.4) 
    time.sleep(0.5)

    uservo.set_servo_angle(3, 28) 
    time.sleep(0.5)

    uservo.set_servo_angle(1, 40.3)

    time.sleep(0.5)
    uservo.set_servo_angle(5,-10)
    time.sleep(2)
 
def catch_num_six():
    uservo.set_servo_angle(2, 10)

    #uservo.set_servo_angle(3, -40)
    time.sleep(0.5)

    uservo.set_servo_angle(0, -100.4) 
    time.sleep(0.5)

    uservo.set_servo_angle(3, 5) 
    time.sleep(0.5)

    uservo.set_servo_angle(1, 55.3)

    time.sleep(0.5)
    uservo.set_servo_angle(5,-10)
    time.sleep(2)

def put_front_right():
    uservo.set_servo_angle(5,-10)
    time.sleep(0.5)
    uservo.set_servo_angle(1,0)
    #put down 2
    uservo.set_servo_angle(0,-50)
    time.sleep(0.5)
    uservo.set_servo_angle(1,-12)
    time.sleep(0.5)
    uservo.set_servo_angle(2,95.5)
    time.sleep(0.5)
    uservo.set_servo_angle(3,16)
    time.sleep(0.5)
    uservo.set_servo_angle(5,80)
    time.sleep(1)

def put_front_left():
    #put down
    uservo.set_servo_angle(0,-110)
    time.sleep(0.5)
    uservo.set_servo_angle(1,-8.3)
    time.sleep(0.5)
    uservo.set_servo_angle(2,90)
    time.sleep(0.5)
    uservo.set_servo_angle(3,5.9)
    time.sleep(0.5)
    uservo.set_servo_angle(5,80)
    time.sleep(0.5)

def put_behind_right():
    #put down
    uservo.set_servo_angle(1,0)
    time.sleep(2)
    uservo.set_servo_angle(0,72.2)
    time.sleep(1)
    uservo.set_servo_angle(1,0)
    time.sleep(1)
    uservo.set_servo_angle(2,90)
    time.sleep(0.5)
    uservo.set_servo_angle(3,20)
    time.sleep(0.5)
    uservo.set_servo_angle(5,80)
    time.sleep(0.5)

def put_behind_left():
    #put down
    uservo.set_servo_angle(2,20)
    time.sleep(0.5)
    uservo.set_servo_angle(1,0)
    time.sleep(1)
    uservo.set_servo_angle(0,132)
    time.sleep(1)
    uservo.set_servo_angle(1,0)
    time.sleep(1)
    uservo.set_servo_angle(2,90)
    time.sleep(0.5)
    uservo.set_servo_angle(3,15)
    time.sleep(0.5)
    uservo.set_servo_angle(5,80)
    time.sleep(0.5)

def catch_behind_right():
    #put down
    uservo.set_servo_angle(2,20)
    time.sleep(0.5)
    uservo.set_servo_angle(1,0)
    time.sleep(1)
    uservo.set_servo_angle(0,72.2)
    time.sleep(1)
    uservo.set_servo_angle(2,90)
    time.sleep(0.5)
    uservo.set_servo_angle(3,20)
    time.sleep(0.5)
    uservo.set_servo_angle(5,-10)
    time.sleep(2)
    uservo.set_servo_angle(1,-20)
    time.sleep(1)

def put_B_right():
    uservo.set_servo_angle(0,92.1)
    time.sleep(0.5)
    uservo.set_servo_angle(3,35)
    time.sleep(0.5)
    uservo.set_servo_angle(2,55)
    time.sleep(0.5)
    uservo.set_servo_angle(1,29)
    time.sleep(0.5)
    uservo.set_servo_angle(5,80)
    time.sleep(1)

def catch_front_right():
    #put down 2
    uservo.set_servo_angle(0,-50)
    time.sleep(0.5)
    uservo.set_servo_angle(1,-4.7)
    time.sleep(0.5)
    uservo.set_servo_angle(2,107)
    time.sleep(0.5)
    uservo.set_servo_angle(3,34)
    time.sleep(0.5)
    uservo.set_servo_angle(5,-10)
    time.sleep(2.5)
    uservo.set_servo_angle(1,-40)
    time.sleep(2)

def assemble_right():
    uservo.set_servo_angle(0,89.0)
    time.sleep(1.5)
    uservo.set_servo_angle(4,120)
    uservo.set_servo_angle(3,12)
    time.sleep(0.5)
    uservo.set_servo_angle(2,41.8)
    time.sleep(0.5)
    uservo.set_servo_angle(1,36.1)
    time.sleep(0.5)
    uservo.set_servo_angle(4,-120)
    time.sleep(2)


def init_servo():
    # 参数配置
    # 角度定义
    SERVO_PORT_NAME =  '/dev/ttyUSB0' # 舵机串口号
    # SERVO_PORT_NAME =  'COM1' # 舵机串口号
    SERVO_BAUDRATE = 115200 # 舵机的波特率
    SERVO_ID = 0  # 舵机的ID号

    # # 初始化串口
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

def set_initial_angle():
    uservo.set_servo_angle(0, -80) 
    uservo.set_servo_angle(1, 0) 
    uservo.set_servo_angle(2, 90)
    uservo.set_servo_angle(3, 0) 
    uservo.set_servo_angle(4, -90)
    uservo.set_servo_angle(5, 80)
    time.sleep(1)

# %%
# parameters 
raspi_A_ipv4="192.168.2.2"
raspi_B_ipv4="192.168.2.1"
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

# lower_g=np.array([35, 43, 40])
# upper_g=np.array([ 70, 255, 255])

lower_g=np.array([0, 180, 0])
upper_g=np.array([ 100, 255, 255])

UPPER_BOUND=400#100#130#600
LOWER_BOUND=680#400#400#1700

ONE_Press_Start=17
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
        print(data)
        self.ser.write(data)
    def read(self):
        try:
            # ser.write('1'.encode())
            strin = self.ser.readline().decode('utf-8')  # 获取arduino发送的数据
            if(len(strin)>1):
                print('arduino',strin)
        except UnicodeDecodeError:
            pass
        return strin
    def close(self):
        self.ser.close()

# %%
# init vars 
img=0
rgb=0

# %%
# the client is used for car A 
class Client():
    def __init__(self,server_info):
        # 1. 创建tcp的套接字
        self.server_ip,self.server_port,self.prefix_key=server_info
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 2. 链接服务器
        server_addr = (self.server_ip, self.server_port)
        self.tcp_socket.connect(server_addr)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

    def send(self,send_data):
        # 3. 发送数据/接收数据
        send_data=self.prefix_key+send_data
        self.tcp_socket.send(send_data.encode("utf-8"))
    def close(self):
        # 4. 关闭套接字
        self.tcp_socket.close()
def sendfile(send_data):
    f = open(imgPath+'0.txt', 'w', encoding='gbk')
    f.write(send_data)
    f.close()
    os.system('scp '+imgPath+'0.txt ubuntu@'+rpi_B_info[0]+':'+imgPath)
        # now_time=0
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
def server_recv(client_socket, ip_port,server_info):
    server_ip,server_port,prefix_key=server_info
    while True:
        recv_data = client_socket.recv(1024).decode("utf-8")
        # 如果接收的消息长度不为0，则将其解码输出
        if recv_data:
            print(ip_port+" said")
            # 接收客户端发送过来的请求
            if (recv_data[:4]==prefix_key):
                print(recv_data)
            client_socket.send("收到\n".encode())
        # 当客户端断开连接时，会一直发送''空字符串，所以长度为0已下线
        else:
            print("客户端", ip_port, "已下线")
            client_socket.close()
            break

def server(server_info,client_info):
    server_ip,server_port,prefix_key=server_info
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 2. 插入手机卡(绑定本地信息 bind)
    tcp_server_socket.bind(("", server_port))
    # 3. 将手机设置为正常的 响铃模式(让默认的套接字由主动变为被动 listen)
    tcp_server_socket.listen(8)
    # 4. 等待别人的电话到来(等待客户端的链接 accept)
    new_client_socket, client_addr = tcp_server_socket.accept()
    if (client_addr[0]==client_info[0]):
        t1 = threading.Thread(target=server_recv, args=(new_client_socket, client_addr,server_info))
        # 设置线程守护
        t1.setDaemon(True)
        # 启动线程
        t1.start()

    # 关闭套接字
    # tcp_server_socket.close()

# %%
# def voice(txt,portx="/dev/ttyUSB0",bps=9600,timex=5,):
#     try:
#         # #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
#         # portx="/dev/ttyUSB0"
#         # #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
#         # bps=9600
#         # #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
#         # timex=5
#         # # 打开串口，并得到串口对象
#         ser=serial.Serial(portx,bps,timeout=timex)
#         # 写数据
#         result=ser.write(("<G>"+txt).encode("gbk"))
#         print("写总字节数:",result)
#         ser.close()#关闭串口
#     except Exception as e:
#         print("---异常---：",e)

# %%
def scanQRCode(srcImg):
    # barcode = reader.decode(ImgPAth)
    # srcImg = cv.imread(ImgPAth)
    srcImg = cv.cvtColor(srcImg, cv.COLOR_BGR2GRAY)
    # cv2.imshow("Image", srcImg)
    barcodes = pyzbar.decode(srcImg,symbols=[pyzbar.ZBarSymbol.QRCODE])#QRCODE
    # print(pyzbar.ZBarSymbol.QRCODE)
    debarcode='0'
    qrdecode='0'
    rgb=[0,0,0]
    for barcode in barcodes:
        # barcodeData = barcode.data.encode("gbk")
        debarcode = barcode.data.decode("utf-8")
        
    # debarcode= barcodes[0].data.decode("utf-8")
        if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
            qrdecode='蓝色绿色'
            rgb=[0,1,1]
            break 
        if( debarcode==r'郤｢濶ｲ扈ｿ濶ｲ'):
            qrdecode='红色绿色'
            rgb=[1,1,0]
            break 
        if( debarcode==r'扈ｿ濶ｲ闢晁牡'):
            qrdecode='绿色蓝色'
            rgb=[0,1,1]
            break 
        if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
            qrdecode='红色蓝色'
            rgb=[1,0,1]
            break 
        if( debarcode==r'郤｢濶ｲ闢晁牡'):
            qrdecode='红色蓝色'
            rgb=[1,0,1]
            break 
    return qrdecode,debarcode,rgb


# %%
def now_time():
    ti=time.localtime()[2:6]
    return str(ti[0])+'_'+str(ti[1])+'_'+str(ti[2])+'_'+str(ti[3])

# %%
# img=cv.imread('/home/ubuntu/mutli-communication/pic/0.jpg')
# # img=img[250:680,130:350][:][:]

# # cv.imwrite(imgPath+'0.jpg',img)
# res=judge_color(img)



# %%
# img=catch_picture(imgPath+now_time()+'.jpg')

# %%
def catch_picture(IMG_PATH=None):
    global img
    time.sleep(1)
    #调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
    start_time=time.time()
    # cap=cv.VideoCapture(0)
    # #success=cv.VideoCapture.isOpened(0)
    # success=True
    # if(not success):
    # cap.open()
    while True:
        #从摄像头读取图片
        success,img=cap.read()
        print('video success',success,'\n')
        #转为灰度图片
        # gray=cv.cvtColor(img,cv.COLOR_BGR2RGB)
        #显示摄像头，背景是灰度。
        # cv2.imshow("img",gray)
        #保持画面的持续。
        # k=cv2.waitKey(1)
        end_time=time.time()
        # if end_time-start_time>3 and success:
        if end_time-start_time>0.1 and success:
            if (IMG_PATH!=None):
                cv.imwrite(IMG_PATH,img)
            # cap.release()
            return img
            # cv2.destroyAllWindows()
            break
    return 0
    


# %%
def clip_1(img,rgb):
    # frame=img[UPPER_BOUND:LOWER_BOUND][:][:]
    frame1=img[250:720,150:350][:][:]
    frame2=img[250:720,350:830][:][:]
    cv.imwrite(imgPath+'clip1]1.jpg',frame1)
    cv.imwrite(imgPath+'clip1]2.jpg',frame2)

    pos=np.array([0,0,0])
    
    res=judge_color(frame2)
    print(res)
    pos[2]=np.where(res==np.max(res))[0][0]+1
    res=judge_color(frame1)
    print(res)
    pos[1]=np.where(res==np.max(res))[0][0]+1
    if(pos[1]==pos[2]):
        print('error when detected')
    pos[0]=6-pos[1]-pos[2]
    if(pos[0]>3):
        pos[0]=3
    if(pos[0]<1):
        pos[0]=1
        print('error when detected')
    need=[0,0,0]
    for i in range(3):
        if(rgb[i]==1):
            for j in range(3):
                if (pos[j]==i+1):
                    need[j]=i+1
    print('pos  ',pos,'   |   need ',need,'\n')
    return need
    pass 


# %%

def clip_2(img,rgb):
    frame1=img[250:680,0:150][:][:]
    frame2=img[250:680,130:350][:][:]
    # frame=img[UPPER_BOUND:LOWER_BOUND][:][:]
    cv.imwrite(imgPath+'clip2]1.jpg',frame1)
    cv.imwrite(imgPath+'clip2]2.jpg',frame2)

    pos=np.array([0,0,0])
    
    res=judge_color(frame2)
    print(res)
    pos[2]=np.where(res==np.max(res))[0][0]+1
    res=judge_color(frame1)
    print(res)
    pos[1]=np.where(res==np.max(res))[0][0]+1
    if(pos[1]==pos[2]):
        print('error when detected')
    pos[0]=6-pos[1]-pos[2]
    if(pos[0]>3):
        pos[0]=3
    if(pos[0]<1):
        pos[0]=1
        print('error when detected')
    need=[0,0,0]
    for i in range(3):
        if(rgb[i]==1):
            for j in range(3):
                if (pos[j]==i+1):
                    need[j]=i+1
    print('pos  ',pos,'   |   need ',need,'\n')
    return need
    # r =1 g =2 b =3 不需要抓为0
    pass 


# %%

def judge_color(frame):
    rgb=[0,0,0]
    frame=cv.GaussianBlur(frame,(5,5),0)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    hsv=cv.erode(hsv,None,iterations=2)
    mask_b = cv.inRange(hsv, lower_blue, upper_blue)
    print(mask_b.shape)
    mask_r = cv.inRange(hsv, lower_red, upper_red)
    mask_g = cv.inRange(hsv, lower_g,upper_g)
    res_r = cv.bitwise_and(frame, frame, mask=mask_r)
    res_g = cv.bitwise_and(frame, frame, mask=mask_g)
    res_b = cv.bitwise_and(frame, frame, mask=mask_b)
    cv.imwrite(imgPath+'cr.jpg',res_r)
    cv.imwrite(imgPath+'cg.jpg',res_g)
    cv.imwrite(imgPath+'cb.jpg',res_b)
    kernel=np.ones((5,5),np.uint8)
    mask_b=cv.erode(mask_b,kernel,iterations=2)
    mask_r=cv.erode(mask_r,kernel,iterations=3)
    mask_g=cv.erode(mask_g,kernel,iterations=3)
    # mask_b=1-mask_b
    # mask_r=1-mask_r
    # mask_g=1-mask_g
    X_axis=np.arange(0,frame.shape[1],1)
    # mask=cv.erode(mask,kernel,iterations=1)
    mask_r=mask_r>0
    mask_g=mask_g>0
    mask_b=mask_b>0
    r_sum=(np.sum(np.nonzero(mask_r))-1)
    g_sum=(np.sum(np.nonzero(mask_g))-1)
    b_sum=(np.sum(np.nonzero(mask_b))-1)
    # x_r=(np.multiply(X_axis,mask_r[:]).sum()/r_sum)if r_sum>200 else -1
    # x_g=(np.multiply(X_axis,mask_g[:]).sum()/g_sum)if g_sum>200 else -1
    # x_b=(np.multiply(X_axis,mask_b[:]).sum()/b_sum)if b_sum>200 else -1

    x_r=(r_sum)if r_sum>200 else -1
    x_g=(g_sum)if g_sum>200 else -1
    x_b=(b_sum)if b_sum>200 else -1
    rgb=np.array([x_r,x_g,x_b])
    return rgb
    pass 

def classify_color(srcimg):
    # frame=cv.imread(IMG_PATH)
    # frame=srcimg
    frame=srcimg[UPPER_BOUND:LOWER_BOUND][:][:]
    # 这里还是弄一个决策树或者yolov5s出来比较好
    # 裁剪图片
    cv.imwrite(imgPath+now_time()+'clip.jpg',frame)

    # frame=cv.resize(frame,(480,100))
    # fram=cv2.contrast
    # cv.imshow('Capture', frame)
    # change to hsv model
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask_b = cv.inRange(hsv, lower_blue, upper_blue)
    print(mask_b.shape)
    mask_r = cv.inRange(hsv, lower_red, upper_red)
    mask_g = cv.inRange(hsv, lower_g,upper_g)
    res_r = cv.bitwise_and(frame, frame, mask=mask_r)
    res_g = cv.bitwise_and(frame, frame, mask=mask_g)
    res_b = cv.bitwise_and(frame, frame, mask=mask_b)
    cv.imwrite(imgPath+now_time()+'r.jpg',res_r)
    cv.imwrite(imgPath+now_time()+'g.jpg',res_g)
    cv.imwrite(imgPath+now_time()+'b.jpg',res_b)
    kernel=np.ones((5,5),np.uint8)
    mask_b=cv.erode(mask_b,kernel,iterations=1)
    mask_r=cv.erode(mask_r,kernel,iterations=1)
    mask_g=cv.erode(mask_g,kernel,iterations=1)
    # mask_b=1-mask_b
    # mask_r=1-mask_r
    # mask_g=1-mask_g
    X_axis=np.arange(0,frame.shape[1],1)
    # mask=cv.erode(mask,kernel,iterations=1)
    mask_r=mask_r>0
    mask_g=mask_g>0
    mask_b=mask_b>0
    r_sum=(np.sum(np.nonzero(mask_r))-1)
    g_sum=(np.sum(np.nonzero(mask_g))-1)
    b_sum=(np.sum(np.nonzero(mask_b))-1)
    x_r=(np.multiply(X_axis,mask_r[:]).sum()/r_sum)if r_sum>200 else -1
    x_g=(np.multiply(X_axis,mask_g[:]).sum()/g_sum)if g_sum>200 else -1
    x_b=(np.multiply(X_axis,mask_b[:]).sum()/b_sum)if b_sum>200 else -1
    # res = cv.bitwise_and(frame, frame, mask=mask1)
    # cv.imwrite('det000.jpg',res)
    # # if __name__ == '__main__':
    # while True:
    #     cv.imshow('img', mask)
    #     if cv.waitKey() == ord('q'):
    #         break
    # cv.destroyAllWindows()

    return (x_r,x_g,x_b)

# %%
# #  Blink an LED with the LGPIO library
# #  Uses lgpio library, compatible with kernel 5.11
# #  Author: William 'jawn-smith' Wilson

# import time
# import lgpio

# LED = 23

# # open the gpio chip and set the LED pin as output
# h = lgpio.gpiochip_open(0)
# lgpio.gpio_claim_output(h, LED)

# try:
#     while True:
#         # Turn the GPIO pin on
#         lgpio.gpio_write(h, LED, 1)
#         time.sleep(1)

#         # Turn the GPIO pin off
#         lgpio.gpio_write(h, LED, 0)
#         time.sleep(1)
# except KeyboardInterrupt:
#     lgpio.gpio_write(h, LED, 0)
#     lgpio.gpiochip_close(h)

# %%
def nextProcess():
    time.sleep(1)
    pass 

# %%
# gpio=pigpio.pi()
# gpio.set_mode(ONE_Press_Start,pigpio.INPUT)

# # client=Client(rpi_B_info)
# NotPressed=1
# while(NotPressed):
#     # if(lgpio.gpio_read(h,11)):
#     if(gpio.read(17)):
#         NotPressed=0
#         print('start')
#     pass
# # client.send("00启动")
# # nextProcess()

# %%
# if __name__=='__main__':
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(11,GPIO.IN,pull_up_down=GPIO.PUD_UP)  

# h=lgpio.gpiochip_open(0)
# lgpio.gpio_claim_input(h,11)


# 添加二维码
# cap=cv.VideoCapture(0)
# time.sleep(1)
# gpio=pigpio.pi()
# gpio.set_mode(ONE_Press_Start,pigpio.INPUT)
# NotPressed=1
# while(NotPressed):
#     # if(lgpio.gpio_read(h,11)):
#     if(gpio.read(17)):
#         NotPressed=0
#         print('start')
#     pass
# # client.send("00启动")
# sendfile("00启动")
# first=catch_picture(imgPath+'01.jpg')
# # img=cv.imread('/home/ubuntu/multi-communication/pic/01.jpg')
# time.sleep(3)
# first=catch_picture(imgPath+'01.jpg')
# img=cv.imread('/home/ubuntu/multi-communication/pic/01.jpg')
# print(scanQRCode(img))

# # init_servo()
# # uservo.set_servo_angle(0,-163)
# # clip_2(detectImg,rgb)
# exit()


# img=cv.imread('/home/ubuntu/multi-communication/pic/clip1.jpg')
# # print(judge_color(img))
# clip_1(img,[1,0,1])
# exit()

vi_num=int(os.popen("ls -l /dev | grep detec_video").readlines()[0][-2:-1])
print("video",vi_num,'\n')
cap=cv.VideoCapture(vi_num-1)


first=catch_picture(imgPath+'01.jpg')
time.sleep(1)
# first=catch_picture(imgPath+'01.jpg')
# time.sleep(2)
# # first=catch_picture(imgPath+'01.jpg')
gpio=pigpio.pi()
gpio.set_mode(ONE_Press_Start,pigpio.INPUT)

# client=Client(rpi_B_info)

arduino=Arduino_communicate()



# 一键启动
NotPressed=1
while(NotPressed):
    # if(lgpio.gpio_read(h,11)):
    if(gpio.read(17)):
        NotPressed=0
        print('start')
    pass
# client.send("00启动")
sendfile("00启动")
arduino.write(b'1')
# nextProcess()
init_servo()
set_initial_angle()

# 扫码


notDetected=1
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass 


# time.sleep(1)
qrimg=catch_picture(imgPath+'_qr.jpg')
rgb=[0,0,0]
text,_,rgb=scanQRCode(qrimg)
print(text,rgb)



# client.send("01"+text)
sendfile("01"+text)
if(text=='0'):
    text='绿色蓝色'
    rgb=[0,1,1]
    
# 扫完码离开
arduino.write(b'1')

# 仓库A 
notDetected=1
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass 
detectImg=catch_picture(imgPath+'clip1.jpg')
    # r =1 g =2 b =3 不需要抓为0
need_catchA=clip_1(detectImg,rgb)

arduino.write(b'1')
# 向前，抵住泡沫
notDetected=1
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass 

# 爪子



if(need_catchA[2]>0):
    catch_num_one()
    put_front_left()

if(need_catchA[1]>0):
    catch_num_two()
    if(need_catchA[2]>0):
        put_front_right()
    else:
        put_front_left()




# time.sleep(1)
# # 爪子结束，换一个位置装配
# arduino.write(b'1')
# time.sleep(1)
# arduino.write(b'1')
# time.sleep(1)
# arduino.write(b'1')


notDetected=1
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass 
# 向左，抵住泡沫

#机械臂抓取
if(need_catchA[0]>0):
    catch_num_three()
    put_front_right()

# time.sleep(1)
# arduino.write(b'1')
# # 爪子结束，前往B区域
uservo.set_servo_angle(0,-163)
notDetected=1
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass 


detectImg=catch_picture(imgPath+'clip2.jpg')
need_catchB=clip_2(detectImg,rgb)
arduino.write(b'1')
# 向前，抵住泡沫



notDetected=1
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass 
# 爪子
if(need_catchB[2]>0):
    catch_num_four()
    if(need_catchA[2]==need_catchB[2]):
        put_behind_left()
    elif(need_catchA[1]==need_catchB[2]):
        if(need_catchA[2]>0):
            put_behind_right()
        else:
            put_behind_left()
    else:
        put_behind_right()


time.sleep(1)
# 爪子结束，换一个位置装配
arduino.write(b'1')
while(notDetected):
    if arduino.read()[:4]=='2002':
        notDetected=0
    pass
if(need_catchB[1]>0):
    catch_num_five()
    if(need_catchA[2]==need_catchB[1]):
        put_behind_left()
    elif(need_catchA[1]==need_catchB[1]):
        if(need_catchA[2]>0):
            put_behind_right()
        else:
            put_behind_left()
    else:
        put_behind_right()

if(need_catchB[0]>0):
    catch_num_six()
    if(need_catchA[2]==need_catchB[0]):
        put_behind_left()
    elif(need_catchA[1]==need_catchB[0]):
        if(need_catchA[2]>0):
            put_behind_right()
        else:
            put_behind_left()
    else:
        put_behind_right()
time.sleep(1)
# 爪子结束，前往装配区域
arduino.write(b'1')

# 告诉B车爪子要夹住某个部分



time.sleep(2)
sendfile("022")
# client.send("022")

time.sleep(2)
sendfile("021")
# client.send("021")


time.sleep(1)
# client.close()
# lgpio.gpiochip_close(h)


# %% [markdown]
# ubuntu@rpi-A:~/mutli-communication$ sudo chown ubuntu /dev/gpiomem
# ubuntu@rpi-A:~/mutli-communication$ sudo chown ubuntu /dev/gpiochip0
# ubuntu@rpi-A:~/mutli-communication$ sudo chown ubuntu /dev/gpiochip1

# %% [markdown]
# https://blog.csdn.net/u010169607/article/details/111316629


