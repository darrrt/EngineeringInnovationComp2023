#pip install adafruit-pca9685
import Adafruit_PCA9685
import time             
# import adapip
# 0：240正后 40正前
# 1： 10上 300下
SERVO0_0=240
SERVO0_1=40
SERVO1_0=5
SERVO1_1=300
SERVO2_0=180
SERVO2_1=0
SERVO3_0=150
SERVO3_1=175
SERVO0=4
SERVO1=1
SERVO2=2
SERVO3=3

#这里只给函数不讲原理了，大部分人只是用不用懂。要了解原理，就要涉及电信号的时差和角度精度等知识。
#想了解的可以看板卡的原理说明书。
def ServoControl(angle,channel):
	date=int(4096*((angle*6.67)+500)/(3030)+0.5)	
	pwm.set_pwm(channel, 0, date)
def setCatchAngel1():
	ServoControl(SERVO0_0,SERVO0)
	ServoControl(SERVO1_0,SERVO1)
	ServoControl(SERVO2_0,SERVO2)
	ServoControl(SERVO3_0,SERVO3)
	for i in range(SERVO0_0,SERVO0_1,-2):
		ServoControl(i,SERVO0)
		time.sleep(0.02)
	# time.sleep(3)
	for i in range(SERVO2_0,SERVO2_1,-2):
		ServoControl(i,SERVO2)
		time.sleep(0.02)
	for i in range(SERVO1_0,SERVO1_1,2):
		ServoControl(i,SERVO1)
		time.sleep(0.02)
def CatchThing():
	for i in range(SERVO3_0,SERVO3_1,5):
		ServoControl(i,SERVO3)
def setPlaceAngel1():
	ServoControl(SERVO0_1,SERVO0)
	ServoControl(SERVO1_1,SERVO1)
	ServoControl(SERVO2_1,SERVO2)
	ServoControl(SERVO3_1,SERVO3)
	for i in range(SERVO1_1,SERVO1_0,-2):
		ServoControl(i,SERVO1)
	for i in range(SERVO2_1,SERVO2_0,2):
		ServoControl(i,SERVO2)
	for i in range(SERVO0_1,SERVO0_0,5):
		ServoControl(i,SERVO0)
def PlaceThing():
	for i in range(SERVO3_1,SERVO3_0,-5):
		ServoControl(i,SERVO3)
if __name__ == '__main__':
	pwm = Adafruit_PCA9685.PCA9685()
	pwm.set_pwm_freq(330) #频率
	# for i in range(0,300,5):
	# 	ServoControl(i,SERVO0)
	# 	print(i)
	# ServoControl(5,SERVO1)
	# ServoControl(0,SERVO2)
	setCatchAngel1()
	# setPlaceAngel1()
	
