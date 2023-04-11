#pip install adafruit-pca9685
import Adafruit_PCA9685             
# import adapip
SERVO0_0=330
SERVO1_0=0
SERVO2_0=200
SERVO3_0=150
SERVO0=4
SERVO1=1
SERVO2=2
SERVO3=3

#这里只给函数不讲原理了，大部分人只是用不用懂。要了解原理，就要涉及电信号的时差和角度精度等知识。
#想了解的可以看板卡的原理说明书。
def ServoControl(angle,channel):
	date=int(4096*((angle*11)+500)/(20000)+0.5)	
	pwm.set_pwm(channel, 0, date)
def setCatchAngel1():
	for i in range(SERVO0_0,SERVO0_0-210,-2):
		ServoControl(i,SERVO0)
	for i in range(SERVO2_0,SERVO2_0-170,-2):
		ServoControl(i,SERVO2)
	for i in range(SERVO1_0,SERVO1_0+180,2):
		ServoControl(i,SERVO1)
def CatchThing():
	for i in range(SERVO3_0,SERVO3_0+25,5):
		ServoControl(i,SERVO3)
def setPlaceAngel1():
	for i in range(SERVO1_0+180,SERVO1_0,-2):
		ServoControl(i,SERVO1)
	for i in range(SERVO2_0-170,SERVO2_0,2):
		ServoControl(i,SERVO2)
	for i in range(SERVO0_0-210,SERVO0_0,5):
		ServoControl(i,SERVO0)
def PlaceThing():
	for i in range(SERVO3_0+25,SERVO3_0,-5):
		ServoControl(i,SERVO3)
if __name__ == '__main__':
	pwm = Adafruit_PCA9685.PCA9685()
	pwm.set_pwm_freq(50) #频率
	ServoControl(0,SERVO0)
	
