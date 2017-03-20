#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool


def detect(img):
    cascade = cv2.CascadeClassifier("cascade.xml")
    rects = cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))

    if len(rects) == 0:
        return [], img
    rects[:, 2:] += rects[:, :2]
    return rects, img

def box(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    #cv2.imwrite('one.jpg', img);

def distance(rects, img):
    #cans have height of 10.47 cm and width of 5.24 cm
    #The can is just slightly in front of the claw at 17.78 cm from the camera.
    height_cm = 10.47 #height of the can
    height_px = 203.0 #can height in pixels at 17.78 cm (7 inches), tbd experimentally
    ref_dist = 20.32 #the known distance from camera for calibration
    px_to_cm = float(height_cm/height_px) #multiply by px to get measurement in cm
    focal_length = (height_px*ref_dist)/height_cm


    for x1, y1, x2, y2 in rects:
        height = abs(y1-y2)
        distance = (height_cm*focal_length)/height_px
        print height
        # print distance
        # if 16.0 < distance < 20.0
        #     return True #pick up the can.
        #http://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/



def talker(coke_can):
    pub = rospy.Publisher('img_rec', Bool, queue_size=10)
    rospy.init_node('img_rec', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = coke_can % rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        break

cap = cv2.VideoCapture(1)
cap.set(3,400)
cap.set(4,300)




while(True):
    ret, img = cap.read()
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower mask (0-10)
    lower_red = np.array([0,160,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,160,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0+mask1

    # set my output img to zero everywhere except my mask
    output_img = img.copy()
    output_img[np.where(mask==0)] = 0

    rects, img = detect(output_img)

    if len(rects) == 0:
        try:
            talker(0)
        except rospy.ROSInterruptException:
            pass
        pass
    else:
        box(rects, output_img)
        distance(rects, output_img)
        try:
            talker(1)
        except rospy.ROSInterruptException:
            pass

    cv2.imshow("frame", output_img)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
	break
