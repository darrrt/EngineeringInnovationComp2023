# import bluetooth
import time
import serial  # 导入模块
import zxing
import numpy as np
import time
import cv2 as cv
import os
import pyzbar.pyzbar as pyzbar
import RPi.GPIO as GPIO   #先要导入模块
# 17 27 22 从近到远 1 2 3
##BCM 对应 GPIO numbers , BOARD 对应 physical numbers。 
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
# GPIO.setmode(2,GPIO.OUT)    #把引脚 2  设置为输出模式
# GPIO.setup(15,GPIO.IN,pull_up_down=GPIO.PUD_UP)     #把引脚 3 设置为输入模式
# GPIO.setup(18,GPIO.OUT)

# GPIO.setup(17,GPIO.OUT)
# GPIO.setup(27,GPIO.OUT)
# GPIO.setup(22,GPIO.OUT)
pos=[17,27,22]
reader = zxing.BarCodeReader()
log_counter = 0
start_time = time.time()
HOST_IP='10.186.61.94'
IMG_PATH='/home/pi/Documents/gx/0.jpg'
# set blue thresh 设置HSV中蓝色、天蓝色范围
lower_red = np.array([0,43,46])
upper_red = np.array([15,255,255])
# lower_blue
lower_blue=np.array([95, 43, 46])
upper_blue=np.array([ 124, 255, 255])

lower_g=np.array([35, 43, 46])
upper_g=np.array([ 70, 255, 255])

UPPER_BOUND=200#130#600
LOWER_BOUND=480#400#1700
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
    # print(barcode.raw)
    print(barcode.parsed)
    return barcode.parsed



# reader = zxing.BarCodeReader()
# log_counter = 0
# time_start = time.time()


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
    cap=cv2.VideoCapture(0)
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
        if end_time-start_time>0.5:
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
    # fram=cv2.contrast
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
# def catch_

# HOST_MAC_ADDR = 'DC:A6:32:AA:75:A3'  # 目的蓝牙的地址
# sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 4097
qr_ed=0
catch_ed1=0
catch_ed2=0
if __name__ == "__main__":
    log=open('log.log', 'w', encoding='utf-8')
    # try:
    # print(classify_color())
    # exit()
    # catch_picture()
    # scanQRCode(IMG_PATH)
    # exit()
    # GPIO.output(17,False)
    # for i in range(3):
    #     GPIO.output(pos[i],False)
    # print(classify_color())
    # print(scanQRCode(IMG_PATH))
    # exit()
    # sock.connect((HOST_MAC_ADDR, port))  # 连接蓝牙
    logPrinter('waiting press')
    
    # while(GPIO.input(15)):
    #     pass
    start_time = time.time()
    logPrinter('开始')
    # log.write(str())
    # GPIO.output(17,True)
    f = open('0.txt', 'w', encoding='gbk')
    f.write(('<A>'+'start'))
    f.close()
    now_time=0
    os.system('scp /home/pi/Documents/gx/0.txt pi@'+HOST_IP+':~/Documents/gx')
    while True :
        now_time=time.time()
        now_time-=start_time
        logPrinter('now time')
        # 使用计时或者串口回传
        if now_time<1:
            # logPrinter('直线')
            pass
        elif 4.5<now_time and now_time<12 and qr_ed==0:
            # GPIO.output(17,False)
            logPrinter('拍二维码')
            catch_picture()
            
            try:
                # sock.send(('<G>'+scanQRCode(IMG_PATH)).encode())
                f = open('0.txt', 'w', encoding='gbk')
                logPrinter('识别二维码')
                try:
                    # 先不要了？# qrdecode=scanQRCode(IMG_PATH)
                    srcImg = cv.imread(IMG_PATH)
                    srcImg = cv.cvtColor(srcImg, cv.COLOR_BGR2GRAY)
                    # cv2.imshow("Image", srcImg)
                    barcodes = pyzbar.decode(srcImg,symbols=[pyzbar.ZBarSymbol.QRCODE])#QRCODE
                    # print(pyzbar.ZBarSymbol.QRCODE)
                    debarcode=0
                    qrdecode=0
                    for barcode in barcodes:
                        # barcodeData = barcode.data.encode("gbk")
                        debarcode = barcode.data.decode("utf-8")
                    # debarcode= barcodes[0].data.decode("utf-8")
                    if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
                        qrdecode='蓝色绿色'
                    if( debarcode==r'郤｢濶ｲ扈ｿ濶ｲ'):
                        qrdecode='红色绿色'
                    if( debarcode==r'扈ｿ濶ｲ闢晁牡'):
                        qrdecode='绿色蓝色'
                    if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
                        qrdecode='红色蓝色'
                    # if( debarcode==r'郤｢濶ｲ闢晁牡'):
                    #     qrdecode='蓝色绿色'
                    # if( debarcode==r'闢晁牡扈ｿ濶ｲ'):
                    #     qrdecode='蓝色绿色'
                    # ZBarSymbol.QRCODE
                    # lanlv
                    # b'\xe9\x97\xa2\xe6\x99\x81\xe7\x89\xa1\xe6\x89\x88\xef\xbd\xbf\xe6\xbf\xb6\xef\xbd\xb2'
                    # 闢晁牡扈ｿ濶ｲ
                    # ZBarSymbol.QRCODE
                    # honglv
                    # b'\xe9\x83\xa4\xef\xbd\xa2\xe6\xbf\xb6\xef\xbd\xb2\xe6\x89\x88\xef\xbd\xbf\xe6\xbf\xb6\xef\xbd\xb2'
                    # 郤｢濶ｲ扈ｿ濶ｲ
                    # ZBarSymbol.QRCODE
                    # lvlan
                    # b'\xe6\x89\x88\xef\xbd\xbf\xe6\xbf\xb6\xef\xbd\xb2\xe9\x97\xa2\xe6\x99\x81\xe7\x89\xa1'
                    # 扈ｿ濶ｲ闢晁牡
                    # ZBarSymbol.QRCODE
                    # honglan
                    # b'\xe9\x83\xa4\xef\xbd\xa2\xe6\xbf\xb6\xef\xbd\xb2\xe9\x97\xa2\xe6\x99\x81\xe7\x89\xa1'
                    # 郤｢濶ｲ闢晁牡
                    # ZBarSymbol.QRCODE
                    # lvlan2
                    # b'\xe6\x89\x88\xef\xbd\xbf\xe6\xbf\xb6\xef\xbd\xb2\xe9\x97\xa2\xe6\x99\x81\xe7\x89\xa1'
                    # 扈ｿ濶ｲ闢晁牡
                    logPrinter(qrdecode)
                    log.write(now_time)
                    log.write(qrdecode)
                    
                    if(qrdecode):
                        f.write(('<G>'+qrdecode))
                        qr_ed=1
                        rgb=[0,0,0]
                        if('绿色'in qrdecode):
                            rgb[1]=1
                        if('红色'in qrdecode):
                            rgb[0]=1
                        if('蓝色'in qrdecode):
                            rgb[2]=1
                except:
                    srcImg = cv.imread(IMG_PATH)
                    srcImg = cv.cvtColor(srcImg, cv.COLOR_BGR2GRAY)
                    # cv2.imshow("Image", srcImg)
                    barcodes = pyzbar.decode(srcImg,symbols=[pyzbar.ZBarSymbol.QRCODE])#QRCODE
                    print(pyzbar.ZBarSymbol.QRCODE)
                    pass
                f.close()
                os.system('scp /home/pi/Documents/gx/0.txt pi@'+HOST_IP+':~/Documents/gx')
            except:
                pass
        elif 17<now_time and now_time<30 and catch_ed1==0:
            logPrinter('rgb:'+str(rgb))
            logPrinter('拍物品1')
            catch_picture()
            logPrinter('识别物品')
            try:
                order=np.array(classify_color())
                logPrinter('order  '+str(order))
                if(np.sum(np.nonzero(order>-1))>=2):
                    catch_ed1=1
                    for i in range(3):
                        if(rgb[i]):
                            # GPIO.output(pos[np.sum(np.nonzero((order-order[i])>0))],True)
                            logPrinter('color   '+str(rgb[i])+'|  pos   '+str(np.sum(np.nonzero((order-order[i])>0)))+'|   1') 
                            log.write(now_time)
                            log.write('color   '+str(rgb[i])+'|  pos   '+str(np.sum(np.nonzero((order-order[i])>0)))+'|   1') 
            except:
                pass
        elif 48<now_time and now_time<50:
            pass
            # for i in range(3):
            #     GPIO.output(pos[i],False)
        elif 50<now_time and now_time<60 and catch_ed2==0:
            logPrinter('拍物品2')
            catch_picture()
            logPrinter('识别物品')
            try:
                order=np.array(classify_color())
                logPrinter('order  '+str(order))
                if(np.sum(np.nonzero(order>-1))>=2):
                    catch_ed1=1
                    for i in range(3):
                        if(rgb[i]):
                            # GPIO.output(pos[np.sum(np.nonzero((order-order[i])>0))],True)
                            logPrinter('color   '+str(rgb[i])+'|  pos   '+str(np.sum(np.nonzero((order-order[i])>0)))+'|   1') 
                            log.write(now_time)
                            log.write('color   '+str(rgb[i])+'|  pos   '+str(np.sum(np.nonzero((order-order[i])>0)))+'|   1') 
            except:
                pass
            
        # elif now_time<30:
        #     logPrinter('直线')
        # elif now_time<45:
        #     logPrinter('夹取')
        #     color=classify_color()
        elif now_time>1000:
            break
        # sock.send('hello!'.encode()) #每隔三秒发送一个字符串
            # time.sleep(3)
    # except:
    #     sock.close()
    #     logPrinter('disconnect')
    # sock.close()
    logPrinter('disconnect')
    logPrinter('task finished')
    log.close()
# if __name__ == "__main__":
#     reader = zxing.BarCodeReader()
#     log_counter=0
#     time_start=time.time()
#     # voice("绿色蓝色")
#     logPrinter("000") 


