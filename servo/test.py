import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()   
def set_servo_angle(channel, angle):                
    date=4096*((angle*11)+500)/20000              
    date=int(4096*((angle*11)+500)/(20000)+0.5)     
    pwm.set_pwm(channel, 0, date) 
set_servo_angle(0, 20) 
