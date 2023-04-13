#pip install adafruit-pca9685
# import cv2
import Adafruit_PCA9685
import time

# import adapip
# 0：130正后
# 1： 10上 300下
SERVO0_0m=262
SERVO0_0l=235
SERVO0_0r=290
SERVO0_1=75
SERVO0_2=20
SERVO0_3=118
SERVO1_0=5
SERVO1_1=300
SERVO1_2=150
SERVO2_0=260
SERVO2_1=90
SERVO2_2=180
SERVO3_0=150
SERVO3_1=185
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

def QRScan(IMG_PATH):
#读取摄像头视频
    img = cv2.imread(IMG_PATH)
    #循环设备图片中的二维码
    for barcode in decode(img):
        #打印出二维码信息
        # print(barcode.data)
        # print(barcode.rect)
        myData=[]
        myData = barcode.data.decode('utf-8')
        print(myData)
    # cv2.imshow('result', img)
    return myData

def catch_picture(IMG_PATH):
    #调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
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
        cv2.imwrite(IMG_PATH,img)
        break
    return


#这里只给函数不讲原理了，大部分人只是用不用懂。要了解原理，就要涉及电信号的时差和角度精度等知识。
#想了解的可以看板卡的原理说明书。
def ServoControl(angle,channel):
	date=int(4096*((angle*6.6666666)+500)/(3030)+0.5)	
	pwm.set_pwm(channel, 0, date)
def CatchThing(angle_before,angle_after):
	for i in range(angle_before,angle_after,5):
		ServoControl(i,SERVO3)
		time.sleep(0.02)
def PlaceThing(angle_before,angle_after):
	for i in range(angle_before,angle_after,-2):
		ServoControl(i,SERVO3)
		time.sleep(0.02)
def setCatchAngel0(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO1_0,SERVO1_0+20,2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
	for i in range(SERVO2_0,SERVO2_2,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_0,SERVO0_4,-1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	PlaceThing(SERVO3_0,SERVO3_2)
	for i in range(SERVO2_2,SERVO2_0-10,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
def setPlaceAngel0(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_4,SERVO0)
	ServoControl(SERVO1_0+20,SERVO1)
	ServoControl(SERVO2_0-10,SERVO2)
	ServoControl(SERVO3_2,SERVO3)
	CatchThing(SERVO3_2,SERVO3_1)
	for i in range(SERVO2_0-10,SERVO2_2-20,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_4,SERVO0_0,1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	for i in range(SERVO1_0+20,SERVO1_0,-2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
	for i in range(SERVO2_2-20,SERVO2_0,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	PlaceThing(SERVO3_1,SERVO3_0)
def setCatchAngel1(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO0_0,SERVO0_4,-1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	# time.sleep(3)
	for i in range(SERVO2_0,SERVO2_1,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO1_0,SERVO1_1,2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
def setPlaceAngel1(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_4,SERVO0)
	ServoControl(SERVO1_1,SERVO1)
	ServoControl(SERVO2_1,SERVO2)
	ServoControl(SERVO3_1,SERVO3)
	for i in range(SERVO2_1,SERVO2_0,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO1_1,SERVO1_0,-2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
	for i in range(SERVO0_4,SERVO0_0,1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
def setCatchAngel2(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO2_0,SERVO2_2,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_0,SERVO0_4,-1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	# # time.sleep(3)
	for i in range(SERVO1_0,SERVO1_2,2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
def setPlaceAngel2(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_4,SERVO0)
	ServoControl(SERVO1_2,SERVO1)
	ServoControl(SERVO2_2,SERVO2)
	ServoControl(SERVO3_1,SERVO3)
	for i in range(SERVO1_2,SERVO1_0,-2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
	for i in range(SERVO2_2,SERVO2_0,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_4,SERVO0_0,1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
def setRecognize(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO2_0,SERVO2_2,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_0,SERVO0_4,-1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	for i in range(SERVO2_2,SERVO2_0,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)

def QRposTO(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO0_0,SERVO0_4,-1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	for i in range(SERVO2_0,SERVO2_1,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO1_0,SERVO1_2,2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)

def QRposRE(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_4,SERVO0)
	ServoControl(SERVO1_2,SERVO1)
	ServoControl(SERVO2_1,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO1_2,SERVO1_0+20,-2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
	for i in range(SERVO2_1,SERVO2_2,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_4,SERVO0_0,1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	PlaceThing(SERVO3_0,SERVO3_2)
	for i in range(SERVO2_2,SERVO2_0-10,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)

if __name__ == '__main__':
	pwm = Adafruit_PCA9685.PCA9685()
	pwm.set_pwm_freq(330) #频率
	# for i in range(0,300,1):
	# 	ServoControl(i,15)
	# 	time.sleep(0.1)
	# 	print(i)
	# ServoControl(5,SERVO1)
	# ServoControl(30,SERVO0)

	#识别并从转盘上抓物体
	# QRposTO(SERVO0_0l,0)
	# # catch_picture("0.jpg")
	# # squ=QRScan("0.jpg")
	# # print(squ)
	# QRposRE(SERVO0_2,0)
	# current_color=0
	# i=0
	# while i<3:
	# 	# if current_color!=squ[i]:
	# 	# 	pass
	# 	# else:
	# 	# 	setPlaceAngel0(SERVO0_SET[i],SERVO0_2)
	# 	#   setCatchAngel0(SERVO0_SET[i],SERVO0_2)
	# 	# 	i+=1
	# 	setPlaceAngel0(SERVO0_SET[i],SERVO0_2)
	# 	if i!=2:
	# 		setCatchAngel0(SERVO0_SET[i],SERVO0_2)
	# 	else:
	# 		setRecognize(SERVO0_0r,SERVO0_1)
	# 	i+=1	
	
	#在1区放物体
	# setPlaceAngel2(SERVO0_0l,SERVO0_1)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setCatchAngel2(SERVO0_0l,SERVO0_2)
	# PlaceThing(SERVO3_1,SERVO3_0)

	# setPlaceAngel2(SERVO0_0m,SERVO0_2)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setCatchAngel1(SERVO0_0m,SERVO0_1)
	# PlaceThing(SERVO3_1,SERVO3_0)

	# setPlaceAngel1(SERVO0_0r,SERVO0_1)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setCatchAngel2(SERVO0_0r,SERVO0_3)
	# PlaceThing(SERVO3_1,SERVO3_0)

	#在一区抓物体
	# set_2=[0,0,0]
	# for i in range(4,7):
	# 	SET_2[i-4]=SET_1[squ[i]]

	# setCatchAngel2(SERVO0_0l,SET_2[0])
	# CatchThing(SERVO3_0,SERVO3_1)
	# setPlaceAngel2(SERVO0_0r,SET_2[0])
	# PlaceThing(SERVO3_1,SERVO3_0)
	# time.sleep(1)
	# setCatchAngel1(SERVO0_0r,SET_2[1])
	# CatchThing(SERVO3_0,SERVO3_1)
	# setPlaceAngel1(SERVO0_0m,SET_2[1])
	# PlaceThing(SERVO3_1,SERVO3_0)
	# time.sleep(1)
	# setCatchAngel2(SERVO0_0m,SET_2[2])
	# CatchThing(SERVO3_0,SERVO3_1)
	# setPlaceAngel2(SERVO0_0l,SET_2[2])
	# PlaceThing(SERVO3_1,SERVO3_0)
	# time.sleep(1)

	#在二区放物体
	# setPlaceAngel2(SERVO0_0l,SERVO0_1)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setCatchAngel2(SERVO0_0l,SERVO0_2)
	# PlaceThing(SERVO3_1,SERVO3_0)

	# setPlaceAngel2(SERVO0_0m,SERVO0_2)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setCatchAngel1(SERVO0_0m,SERVO0_1)
	# PlaceThing(SERVO3_1,SERVO3_0)

	# setPlaceAngel1(SERVO0_0r,SERVO0_1)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setCatchAngel2(SERVO0_0r,SERVO0_3)
	# PlaceThing(SERVO3_1,SERVO3_0)


	#调试
	# ServoControl(SERVO0_1,SERVO0)
	# ServoControl(SERVO1_1,SERVO1)
	# ServoControl(SERVO2_1,SERVO2)
	# ServoControl(SERVO3_0,SERVO3)

	ServoControl(SERVO0_0l,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	# ServoControl(0,15)

	
