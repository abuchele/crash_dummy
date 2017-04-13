#opens up a webcam feed so you can then test your classifer in real time
# edited by CS 4/11/2017
# Publishes:
#     (Bool)    /can/flag           whether can is present
#     (Float)   /can/distance       distance of can in cm
#     (Bool)    /can/position_flag  whether can is at the correct position to be picked up
#                                   1. if abs(angle) <5
#                                   2. distance ideal (16cm-20cm)
#     (Float)   /can/angle          angle from the centre (-ve to the left 0 in centre, +ve to the right) in degrees
#using detectMultiScale
import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64


#detects can and returns array of rects
def detect(img):
    cascade = cv2.CascadeClassifier("cascade.xml")
    rects = cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))

    if len(rects) == 0:
        return [], img
    rects[:, 2:] += rects[:, :2]
    return rects, img

# find the biggest rect in the rect array provided
# return array of biggest_rect ( positional information for the can)
def box(rects, img):
    max_area = 0
    xbig = 0
    biggest_rect = []
    for x1, y1, x2, y2 in rects:
        area = abs(x1-x2) * abs(y1-y2)
        if abs(x1-x2) * abs(y1-y2) > max_area:
            max_area = area
            biggest_rect = [x1, y1, x2, y2]

    cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    #angle in degrees
    angle = (biggest_rect[0] + (biggest_rect[2]-biggest_rect[0])/2)/320.0 * 63 - 31.5
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
    can_distance.data = (height_cm*focal_length)/height
        # print distance
    if can_distance.data < 30 :
        return True
    return False


## publishes all the data accordingly
def talker(flag_info,distance_info,position_flag_info,angle_info):
    while not rospy.is_shutdown():
        pub_flag.publish(flag_info)
        pub_distance.publish(distance_info)
        pub_position_flag.publish(position_flag_info)
        pub_angle.publish(angle_info)
        break

cap = cv2.VideoCapture(0) #1 for webcam
cap.set(3,400)
cap.set(4,300)

doMask = False # set to True if you want to do the red mask, otherwise False

#ros publisher initialization
pub_flag = rospy.Publisher('can/flag', Bool, queue_size=10)
pub_distance = rospy.Publisher('can/distance', Float64, queue_size=10)
pub_position_flag = rospy.Publisher('can/position_flag', Bool, queue_size=10)
pub_angle = rospy.Publisher('can/angle', Float64, queue_size=10)
rospy.init_node('img_rec', anonymous=True)
rate = rospy.Rate(10) # 10hz

can_count = 0  # counts the number of consecutive can detects
#Float64 distance = 0
#Float64 angle = 0
can_distance = Float64()
angle = Float64()

while(True):

    ret, img = cap.read()

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

        # set output img to zero everywhere except mask
        output_img = img.copy()
        output_img[np.where(mask==0)] = 0

    else:
        output_img = img.copy()

    rects, img_o = detect(output_img)

    if len(rects) == 0: ##if nothing detected
        try:
            talker(0,-1,0,-1)
        except rospy.ROSInterruptException:
            pass
        pass
    else:               # if can detected
        try:
            [biggest_rect, angle.data] = box(rects, img_o)
            print ("angle:", angle)
            action = distance(biggest_rect, img_o) #whether distance of can is ideal for pick up
            print(" ideal distance ",action)
            print(" distance ",can_distance)
            if ((action) & (abs(angle.data) < 5)):
                talker(1,can_distance,1,angle)
            else:
                talker(1,can_distance,0,angle)
        except rospy.ROSInterruptException:
            pass

    cv2.imshow("frame", img_o)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
	       break
