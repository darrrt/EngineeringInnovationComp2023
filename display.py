import RPi.GPIO as GPIO
import time
import math
from PIL import Image

cs = 23  # 片选
rs = 17  # 数据 / 命令 切换
sda = 13  # 数据
scl = 19  # 时钟
reset = 27  # 复位


# 传输byte
def setByteData(data):
    # print ""
    # print "S-----------setByte---------------:", hex(data)
    for bit in range(0, 8):
        # 传入的数字从高位到低位依次判断是否为1，若为1则设置高电平，否则设置低电平
        # 判断的方法是先向左移位，把要判断的位移动到最高位然后跟0x80（1000 0000）相与，
        # 如果结果仍然是0x80（1000 0000）就表示最高位是1，否则最高位就是0
        if ((data << bit) & 0x80 == 0x80):
            setBitData(True)
            # print "1",
        else:
            setBitData(False)
            # print "0",
    # print ""
    # print "E-----------setByte---------------"


def setBitData(data):
    GPIO.output(scl, False)
    GPIO.output(sda, data)
    GPIO.output(scl, True)


def write_command(cmd):
    GPIO.output(cs, False)
    GPIO.output(rs, False)
    setByteData(cmd)
    GPIO.output(cs, True)


def write_data(data):
    GPIO.output(cs, False)
    GPIO.output(rs, True)
    setByteData(data)
    GPIO.output(cs, True)


def write_data_16bit(dataH, dataL):
    write_data(dataH)
    write_data(dataL)


def lcd_reset():
    GPIO.output(reset, False)
    time.sleep(0.1)
    GPIO.output(reset, True)
    time.sleep(0.1)


def lcd_init(width, heigh):
    lcd_reset()

    write_command(0x11)  # Exit Sleep
    time.sleep(0.02)
    write_command(0x26)  # Set Default Gamma
    write_data(0x04)
    write_command(0xB1)  # Set Frame Rate
    write_data(0x0e)
    write_data(0x10)
    write_command(0xC0)  # Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
    write_data(0x08)
    write_data(0x00)
    write_command(0xC1)  # Set BT[2:0] for AVDD & VCL & VGH & VGL
    write_data(0x05)
    write_command(0xC5)  # Set VMH[6:0] & VML[6:0] for VOMH & VCOML
    write_data(0x38)
    write_data(0x40)

    write_command(0x3a)  # Set Color Format
    write_data(0x05)
    write_command(0x36)  # RGB
    write_data(0xc8)

    write_command(0x2A)  # Set Column Address
    write_data(0x00)
    write_data(0x00)
    write_data(0x00)
    write_data(width - 1)
    write_command(0x2B)  # Set Page Address
    write_data(0x00)
    write_data(0x00)
    write_data(0x00)
    write_data(heigh - 1)

    write_command(0xB4)
    write_data(0x00)

    write_command(0xf2)  # Enable Gamma bit
    write_data(0x01)
    write_command(0xE0)
    write_data(0x3f)  # p1
    write_data(0x22)  # p2
    write_data(0x20)  # p3
    write_data(0x30)  # p4
    write_data(0x29)  # p5
    write_data(0x0c)  # p6
    write_data(0x4e)  # p7
    write_data(0xb7)  # p8
    write_data(0x3c)  # p9
    write_data(0x19)  # p10
    write_data(0x22)  # p11
    write_data(0x1e)  # p12
    write_data(0x02)  # p13
    write_data(0x01)  # p14
    write_data(0x00)  # p15
    write_command(0xE1)
    write_data(0x00)  # p1
    write_data(0x1b)  # p2
    write_data(0x1f)  # p3
    write_data(0x0f)  # p4
    write_data(0x16)  # p5
    write_data(0x13)  # p6
    write_data(0x31)  # p7
    write_data(0x84)  # p8
    write_data(0x43)  # p9
    write_data(0x06)  # p10
    write_data(0x1d)  # p11
    write_data(0x21)  # p12
    write_data(0x3d)  # p13
    write_data(0x3e)  # p14
    write_data(0x3f)  # p15

    write_command(0x29)  # Display On
    write_command(0x2C)


def show_single_color(DH, DL, width, heigh):
    for i in range(0, heigh):
        for j in range(0, width):
            write_data_16bit(DH, DL)


def rgb2rgb565(image):
    '''
    将RGB（255，255，255）转换为（31，63，31）
    '''
    rgb565 = []
    w, h = image.size
    for y in range(h):
        for x in range(w):
            r, g, b = image.getpixel((x, y))[:3]
            r = math.ceil(r / 0xFF * 0x1F)
            g = math.ceil(g / 0xFF * 0x3f)
            b = math.ceil(b / 0xFF * 0x1F)
            rgb565.append((r, g, b))

    return rgb565


def show4rgb565(rgb565):
    '''
    将rgb合并后取高低8位用于传输显示
    原文中的RGB565 从高位到低位依次是红、绿、蓝
    但是我所用的屏幕需要蓝、绿、红才能正常显示
    '''
    DH = 0
    DL = 0
    for r, g, b in rgb565:
        h16 = ((b << 6) + g << 5) + r # 蓝、绿、红
        # h16 = ((r << 6) + g << 5) + b # 红、绿、蓝
        DH = (h16 & 0xFF00) >> 8
        DL = h16 & 0x00FF

        write_data_16bit(DH, DL)
    # print(hex(h16),hex(DH), hex(DL))


if __name__ == '__main__':
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cs, GPIO.OUT)
        GPIO.setup(rs, GPIO.OUT)
        GPIO.setup(sda, GPIO.OUT)
        GPIO.setup(scl, GPIO.OUT)
        GPIO.setup(reset, GPIO.OUT)

        lcd_init(128, 160) # 屏幕宽、高
        write_command(0x2C)
        image = Image.open('图片路径')
        image = image.resize((128, 160), Image.ANTIALIAS) #修改图片大小 屏幕宽、高
        rgb565 = rgb2rgb565(image)
        show4rgb565(rgb565)

        while True:
            pass

    except KeyboardInterrupt:
        pass

    # 清理GPIO口
    GPIO.cleanup()
