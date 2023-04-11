import serial #导入模块
import time
def voice(txt,portx="/dev/ttyUSB0",bps=9600,timex=5,):
    try:
        # #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
        # portx="/dev/ttyUSB0"
        # #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        # bps=9600
        # #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        # timex=5
        # # 打开串口，并得到串口对象
        ser=serial.Serial(portx,bps,timeout=timex)
        # 写数据
        result=ser.write(("<G>"+txt).encode("gbk"))
        print("写总字节数:",result)
        ser.close()#关闭串口
    except Exception as e:
        print("---异常---：",e)

# reader = zxing.BarCodeReader()
log_counter = 0
start_time = time.time()
IMG_PATH='000.jpg'

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


# def scanQRCode(ImgPAth):
#     barcode = reader.decode(ImgPAth)
#     print(barcode.raw)
#     print(barcode.parsed)
#     return barcode.parsed


# reader = zxing.BarCodeReader()
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
def catch_picture():
    return 
def if_qrcode():
    return 
def classify_color():
    return
# def catch_
import bluetooth

server_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 1	#设置端口号
server_sock.bind(("", port))	#绑定地址和端口
server_sock.listen(1)	#绑定监听，最大挂起连接数为1
if __name__ =='__main__':
    try:
        while True:
            print('正在等待接收数据。。。')
            client_sock,address=server_sock.accept()  #阻塞等待连接
            print('连接成功')
            print("Accepted connection from ", address)
            while True:
                data =client_sock.recv(1024).decode() #不断接收数据，每次接收缓冲区1024字节
                logPrinter("received [%s]" % data)
                if data[:3]=='<G>':
                    voice(data[3:])
                elif data[:3]=='<A>':#车辆动作
                    pass
                # 这里添加别的动作


    except:
        client_sock.close()
        server_sock.close()
        print('disconnect!')
