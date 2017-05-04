#!/usr/bin/env python

#created by Anna, Max, and ChongSwee in 3/2017

#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

miss_stat_read = 0;

doMask = True # set to True if you want to do the red mask, otherwise False



def detect(img):
    #rects = cascade.detectMultiScale(img, 1.3, 4, cv2.CASCADE_SCALE_IMAGE, (20,20))


    _, conts, hierarchy = cv2.findContours(img.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #adapter code so that it still gives you a rectangle.
    rects = []
    for i in range(len(conts)):
        cont=conts[i]
        x, y, w, h = cv2.boundingRect(cont)
        rect = [x,y,x+w,y+h]
        rects.append(rect)

    if len(rects) == 0:
        return [], img

    else:
        #rects[:, 2:] += rects[:, :2]
        return rects, img

def box(rects, img):
    max_area = 0
    xbig = 0
    xf1 = 0
    xf2 = 0
    yf1 = 0
    yf2 = 0
    biggest_rect = []
    for x1, y1, x2, y2 in rects:
        if (x2-x1) > xbig :
            xbig = x2 - x1
            xf1=x1
            xf2=x2
            yf1=y1
            yf2=y2
    for x1, y1, x2, y2 in rects:
        area = abs(x1-x2) * abs(y1-y2)
        if abs(x1-x2) * abs(y1-y2) > max_area:
            max_area = area
            biggest_rect = [x1, y1, x2, y2]

    cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    angle = (((xf1 + (xf2-xf1)/2)/320.0) * 63 - 33.5)
    cv2.line(img,((xf1+xf2)/2,0),((xf1+xf2)/2,320),(255,255,255),2)
    return [biggest_rect, angle]


def distance(rect, img):
    #cans have height of 10.47 cm and width of 5.24 cm
    #The can is just slightly in front of the claw at 17.78 cm from the camera.
    height_cm = 10.47 #height of the can
    height_px = 203.0 #can height in pixels at 17.78 cm (7 inches), tbd experimentally
    ref_dist = 20.32 #the known distance from camera for calibration
    px_to_cm = float(height_cm/height_px) #multiply by px to get measurement in cm
    focal_length = (height_px*ref_dist)/height_cm

    y1 = rect[1]
    y2 = rect[3]
    height = abs(y1-y2)
    distance = (height_cm*focal_length)/height
    if 27.0 < distance < 47.0 :
        return True
    return False


def talker(coke_can,miss_stat):
    pub = rospy.Publisher('img_rec/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('img_rec/miss_stat', Int8, queue_size=10)
    rospy.init_node('img_rec', anonymous=True)

    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        rospy.loginfo(coke_can)
        pub.publish(coke_can)
        msg.data = miss_stat;
        pub2.publish(msg)
        break

def talker_miss_stat(miss_stat):
    pub2 = rospy.Publisher('/miss_stat', Int8, queue_size=10)
    #rospy.init_node('img_rec_distance', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        msg.data = miss_stat;
        pub2.publish(msg)
        break

def check_status(miss_stat_val):
    miss_stat_read = miss_stat_val.data;


cap = cv2.VideoCapture(0) #1 for webcam

cap.set(3,400)
cap.set(4,300)
twist = Twist()
miss_stat = 1
msg = Int8()

while(True):

    rospy.Subscriber('/miss_stat', Int8, check_status)
    ret, img = cap.read()
    screen = img.copy()

    if doMask:
        img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        # lower mask (0-10)
        lower_red = np.array([0,160,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170,160,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join masks
        mask = mask0+mask1

        #mask = cv2.erode(mask, None, iterations=2)
        #mask = cv2.dilate(mask, None, iterations=2)

        # set output img to zero everywhere except mask
        #output_img = img.copy()
        #output_img[np.where(mask==0)] = 0
        mask = cv2.medianBlur(mask, 5)
        output_img = mask

    else:
        output_img = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)

    rects, img_o = detect(output_img)

    if len(rects) == 0:
        try:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            miss_stat = 1
            talker(twist,miss_stat);
        except rospy.ROSInterruptException:
            pass
        pass
    else:
        try:
            [biggest_rect, angle] = box(rects, img_o)

            angle_new = angle - 3
            cv2.rectangle(screen, (biggest_rect[0], biggest_rect[1]),(biggest_rect[2], biggest_rect[3]), (0,0,0))
            twist.angular.z = angle_new   #(angle/180.0)*100

            action = distance(biggest_rect, img_o)
            if (action):
                if ((abs(angle_new) < 7)):

                    miss_stat = 3
                else:
                    miss_stat = 2
                    twist.linear.x = -20
                    twist.angular.z = 0
                talker(twist,miss_stat);

            else:
                aabs = abs(angle)
                if aabs < 10:
                    twist.linear.x = 12;
                elif aabs < 20:
                    twist.linear.x = 18;
                else:
                    twist.linear.x = 22;
                miss_stat = 2
                talker(twist,miss_stat);
        except rospy.ROSInterruptException:
            pass

    cv2.imshow("frame", img_o)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    if rospy.is_shutdown():
        break
    if miss_stat_read == 5: #mission status 5 means restart, so exit so we can restart
        break
