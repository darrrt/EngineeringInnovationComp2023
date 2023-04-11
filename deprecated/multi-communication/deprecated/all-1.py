import bluetooth
import time
import serial  # 导入模块
import zxing
import numpy as np
import time
import cv2 as cv

reader = zxing.BarCodeReader()
log_counter = 0
start_time = time.time()
IMG_PATH='/home/pi/Documents/gx/0003.jpg'
# set blue thresh 设置HSV中蓝色、天蓝色范围
lower_red = np.array([0,43,46])
upper_red = np.array([15,255,255])
# lower_blue
lower_blue=np.array([95, 43, 46])
upper_blue=np.array([ 124, 255, 255])

lower_g=np.array([35, 43, 46])
upper_g=np.array([ 77, 255, 255])

UPPER_BOUND=600#130#600
LOWER_BOUND=1700#400#1700
def voice(txt, portx="/dev/ttyUSB0", bps=9600, timex=5,):
    try:
        # #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
        # portx="/dev/ttyUSB0"
        # #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        # bps=9600
        # #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        # timex=5
        # # 打开串口，并得到串口对象
        ser = serial.Serial(portx, bps, timeout=timex)
        # 写数据
        result = ser.write(("<G>"+txt).encode("gbk"))
        print("写总字节数:", result)
        ser.close()  # 关闭串口
    except Exception as e:
        print("---异常---：", e)
    return


def scanQRCode(ImgPAth):
    barcode = reader.decode(ImgPAth)
    # logPrinter()
    print(barcode)
    print(barcode.raw)
    print(barcode.parsed)
    return barcode.parsed



reader = zxing.BarCodeReader()
log_counter = 0
time_start = time.time()


def logPrinter(logtxt):
    global log_counter
    # global start_time
    end_time = time.time()
    print("log: %d time %.3f  --  %s" %
          (end_time-start_time, log_counter, logtxt))
    log_counter += 1
    return
    
import numpy as np
import cv2
import time 
def catch_picture():
    #调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
    start_time=time.time()
    cap=cv2.VideoCapture(2)
    while True:
        #从摄像头读取图片
        sucess,img=cap.read()
        #转为灰度图片
        # gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        #显示摄像头，背景是灰度。
        # cv2.imshow("img",gray)
        #保持画面的持续。
        # k=cv2.waitKey(1)
        end_time=time.time()
        if end_time-start_time>2:
            cv2.imwrite(IMG_PATH,img)
            # cv2.destroyAllWindows()
            break
    return 
# def if_qrcode():

#     return 
def classify_color():
    frame=cv.imread(IMG_PATH)
    
    frame=frame[UPPER_BOUND:LOWER_BOUND][:][:]
    frame=cv2.resize(frame,(480,100))
    # cv.imshow('Capture', frame)
    # change to hsv model
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # get mask 利用inRange()函数和HSV模型中蓝色范围的上下界获取mask，mask中原视频中的蓝色部分会被弄成白色，其他部分黑色。

    mask_b = cv.inRange(hsv, lower_blue, upper_blue)
    print(mask_b.shape)
    mask_r = cv.inRange(hsv, lower_red, upper_red)
    mask_g = cv.inRange(hsv, lower_g,upper_g)
    res_r = cv.bitwise_and(frame, frame, mask=mask_r)
    res_g = cv.bitwise_and(frame, frame, mask=mask_g)
    res_b = cv.bitwise_and(frame, frame, mask=mask_b)
    cv.imwrite('/home/pi/Documents/gx/r.jpg',res_r)
    cv.imwrite('/home/pi/Documents/gx/g.jpg',res_g)
    cv.imwrite('/home/pi/Documents/gx/b.jpg',res_b)
    kernel=np.ones((5,5),np.uint8)
    mask_b=cv.erode(mask_b,kernel,iterations=5)
    mask_r=cv.erode(mask_r,kernel,iterations=5)
    mask_g=cv.erode(mask_g,kernel,iterations=5)
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
# def catch_

HOST_MAC_ADDR = 'DC:A6:32:AA:75:A3'  # 目的蓝牙的地址
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 1

if __name__ == "__main__":
    # try:
    print(classify_color())
    # return
#     sock.connect((HOST_MAC_ADDR, port))  # 连接蓝牙
    
#     while True:
#         now_time=time.time()
#         now_time-=start_time
#         print('now time')
#         # 使用计时或者串口回传
#         if now_time<10:
#             logPrinter('直线')
#         elif now_time<15:
#             logPrinter('拍照')
#             sock.send(('<G>'+scanQRCode(IMG_PATH)).encode())
#         elif now_time<30:
#             logPrinter('直线')
#         elif now_time<45:
#             logPrinter('左横')
#         else:
#             break
#         # sock.send('hello!'.encode()) #每隔三秒发送一个字符串
#             # time.sleep(3)
#     # except:
#     #     sock.close()
#     #     logPrinter('disconnect')
#     sock.close()
#     logPrinter('disconnect')
#     logPrinter('task finished')
# # if __name__ == "__main__":
# #     reader = zxing.BarCodeReader()
# #     log_counter=0
# #     time_start=time.time()
# #     # voice("绿色蓝色")
# #     logPrinter("000") 


