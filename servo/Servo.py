#pip install adafruit-pca9685
import Adafruit_PCA9685
import time             
# import adapip
# 0：130正后
# 1： 10上 300下
SERVO0_0m=247
SERVO0_0l=215
SERVO0_0r=285
SERVO0_1=55
SERVO0_2=0
SERVO0_3=98
SERVO1_0=5
SERVO1_1=300
SERVO1_2=150
SERVO2_0=170
SERVO2_1=0
SERVO2_2=90
SERVO3_0=150
SERVO3_1=185
SERVO3_2=120
SERVO0=4
SERVO1=1
SERVO2=2
SERVO3=3

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
	for i in range(SERVO2_0,SERVO2_2,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_0,SERVO0_4,-1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	PlaceThing(SERVO3_0,SERVO3_2)
	for i in range(SERVO2_2,SERVO2_0,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
def setPlaceAngel0(SERVO0_0,SERVO0_4):
	ServoControl(SERVO0_4,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_2,SERVO3)
	CatchThing(SERVO3_2,SERVO3_1)
	for i in range(SERVO2_0,SERVO2_2-20,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO0_4,SERVO0_0,1):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	for i in range(SERVO2_2-20,SERVO2_0,2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	PlaceThing(SERVO3_1,SERVO3_0)
def setCatchAngel1(SERVO0_0):
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO0_0,SERVO0_1,-1):
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
	for i in range(SERVO1_1,SERVO1_0,-2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
	for i in range(SERVO2_1,SERVO2_0,2):
		ServoControl(i,SERVO2)
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

if __name__ == '__main__':
	pwm = Adafruit_PCA9685.PCA9685()
	pwm.set_pwm_freq(330) #频率
	# for i in range(0,300,1):
	# 	ServoControl(i,15)
	# 	time.sleep(0.1)
	# 	print(i)
	# ServoControl(5,SERVO1)
	# ServoControl(30,SERVO0)

	setCatchAngel0(SERVO0_0l,SERVO0_1)
	time.sleep(1)
	setPlaceAngel0(SERVO0_0l,SERVO0_1)

	# setRecognize(SERVO0_0l,SERVO0_1)
	# time.sleep(1)
	# setCatchAngel2(SERVO0_1,SERVO0_2)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setPlaceAngel2(SERVO0_0r,SERVO0_2)
	# PlaceThing(SERVO3_1,SERVO3_0)
	# time.sleep(1)
	# setCatchAngel1(SERVO0_0r)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setPlaceAngel1(SERVO0_0m,SERVO0_1)
	# PlaceThing(SERVO3_1,SERVO3_0)
	# time.sleep(1)
	# setCatchAngel2(SERVO0_0m,SERVO0_3)
	# CatchThing(SERVO3_0,SERVO3_1)
	# setPlaceAngel2(SERVO0_0l,SERVO0_3)
	# PlaceThing(SERVO3_1,SERVO3_0)
	# time.sleep(1)


	# ServoControl(SERVO0_3,SERVO0)
	# ServoControl(SERVO1_0,SERVO1)
	# ServoControl(SERVO2_2,SERVO2)
	# ServoControl(SERVO3_0,SERVO3)
	
