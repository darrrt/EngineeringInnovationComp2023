import cv2
import numpy as np
from pyzbar.pyzbar import decode

# set blue thresh 设置HSV中蓝色、天蓝色范围
lower_red = np.array([0,43,46])
upper_red = np.array([15,255,255])
# lower_blue
lower_blue=np.array([95, 43, 46])
upper_blue=np.array([ 124, 255, 255])

lower_g=np.array([35, 43, 46])
upper_g=np.array([ 70, 255, 255])

IMG_PATH_COLOR="1.jpg"
IMG_PATH_QR="0.jpg"

def classify_color():
    frame=cv2.imread(IMG_PATH_COLOR)
    # frame=frame[UPPER_BOUND:LOWER_BOUND][:][:]
    # frame=cv2.resize(frame,(480,100))
    # fram=cv2.contrast
    # cv.imshow('Capture', frame)
    # change to hsv model
    hsv = cv2.cvtColor(frame, cv.COLOR_BGR2HSV)
    # get mask 利用inRange()函数和HSV模型中蓝色范围的上下界获取mask，mask中原视频中的蓝色部分会被弄成白色，其他部分黑色。
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
    print(mask_b.shape)
    mask_r = cv2.inRange(hsv, lower_red, upper_red)
    mask_g = cv2.inRange(hsv, lower_g,upper_g)
    res_r = cv2.bitwise_and(frame, frame, mask=mask_r)
    res_g = cv2.bitwise_and(frame, frame, mask=mask_g)
    res_b = cv2.bitwise_and(frame, frame, mask=mask_b)
    cv2.imwrite('r.jpg',res_r)
    cv2.imwrite('g.jpg',res_g)
    cv2.imwrite('b.jpg',res_b)
    # kernel=np.ones((5,5),np.uint8)
    # mask_b=cv.erode(mask_b,kernel,iterations=1)
    # mask_r=cv.erode(mask_r,kernel,iterations=1)
    # mask_g=cv.erode(mask_g,kernel,iterations=1)

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

if __name__ == '__main__':
    catch_picture(IMG_PATH_QR)
    QRScan(IMG_PATH_QR)
# mydata=QRScan()
# print(mydata)