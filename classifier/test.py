#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy
import cv2




def detect(img):
    rectsClean = [[0,0,0,0]]
    cascade = cv2.CascadeClassifier("cascade.xml")
    rects = cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    count = 0
    for guess in rects:
        [x1,y1,x2,y2] = guess
        for x in range (x1,x2):
            for y in range (y1,y2):
                [h,s,v] = hsv[x,y]
                print [h,s,v]
                if ((h < 30) or (h>300)):
                    count += 1
                if count > 100:
                    rectsClean += guess
        if count > 100:
            rectsClean += guess
        count = 0
    if len(rectsClean) == 0:
        return [], img
    #rectsClean[:, 2:] += rectsClean[:, :2]
    return rectsClean, img

def box(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    #cv2.imwrite('one.jpg', img);

cap = cv2.VideoCapture(0)
cap.set(3,400)
cap.set(4,300)

while(True):
    ret, img = cap.read()
    rects, img = detect(img)
    if rects == [[]]:
        pass
    else:
        box(rects, img)
    cv2.imshow("frame", img)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
	break