#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import sys


def detect(img):
    cascade = cv2.CascadeClassifier("cascade.xml")
    rects = cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))

    if len(rects) == 0:
        return [], img
    rects[:, 2:] += rects[:, :2]
    return rects, img

def box(rects, img):
    xbig = 0 # stores the biggest box x length
    # variables to store biggest box's positions
    xf1 = 0
    xf2 = 0
    yf1 = 0
    yf2 = 0
    # look for the biggest box
    for x1, y1, x2, y2 in rects:
	if (x2-x1) > xbig :
		xbig = x2 - x1
		xf1 = x1
		xf2 = x2
		yf1 = y1
		yf2 = y2
    # draw line and box for can.
    cv2.rectangle(img, (xf1, yf1), (xf2, yf2), (127, 255, 2), 2)
    cv2.line(img,((xf1+xf2)/2,0),((xf1+xf2)/2,320),(255,255,255),2)

    angle = (xf1 + (xf2-xf1)/2)/320.0 * 63 - 31.5
    return angle

def talker(coke_can):
    pub = rospy.Publisher('can_vel', Twist, queue_size=10)
    rospy.init_node('img_rec', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(coke_can)
        pub.publish(coke_can)
        break

cap = cv2.VideoCapture(0)
#Set resolution.
cap.set(4,300)
cap.set(3,400)
# twist info to be published into "can_vel"
twist = Twist()



while(True):

    ret, img = cap.read()
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower mask (0-10)
    lower_red = np.array([0,160,150])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,160,150])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0 + mask1

    # set my output img to zero everywhere except my mask
    output_img = img.copy()
    output_img[np.where(mask==0)] = 0
    output_img[np.where(mask!=0)] = 255

    rects, img = detect(output_img)
    #cv2.line(img,(100,0),(100,320),(255,255,255),2)
    #cv2.line(img,(310,0),(310,320),(255,255,255),2)

    if len(rects) == 0:
        try:
	    pass
        #if does not contain any rect, return 0 for vel and angle
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            talker(twist)
        except rospy.ROSInterruptException:
            pass
        pass
    else:
        angle = box(rects, output_img)
        print ("angle:", angle);
            try:
                pass
                # set angle(in radians)
        	    twist.angular.z = (angle/180.0)*3.142
                twist.linear.x = 0.1 # change vel if required, 0.1 is just for testing
                talker(twist)
                except rospy.ROSInterruptException:
                        pass

    cv2.imshow("frame", output_img)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
	break
