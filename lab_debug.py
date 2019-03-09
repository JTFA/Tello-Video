import cv2
import numpy as np

def nothing(x):
    pass
#def lab_debug(img):

img = cv2.imread('img/10.jpg')

cv2.namedWindow('image')
cv2.createTrackbar('lmin','image',0,255,nothing)

cv2.createTrackbar('lmax','image',0,255,nothing)


cv2.createTrackbar('amin','image',0,255,nothing)

cv2.createTrackbar('amax','image',0,255,nothing)


cv2.createTrackbar('bmin','image',0,255,nothing)

cv2.createTrackbar('bmax','image',0,255,nothing)

cv2.imshow('capture', img)
while 1:
    if cv2.waitKey(1) == 27:
        break

    lmin = cv2.getTrackbarPos('lmin','image')
    lmax = cv2.getTrackbarPos('lmax','image')

    amin = cv2.getTrackbarPos('amin', 'image')
    amax = cv2.getTrackbarPos('amax', 'image')

    bmin = cv2.getTrackbarPos('bmin', 'image')
    bmax = cv2.getTrackbarPos('bmax', 'image')

    redLower = np.array([lmin, amin, bmin])
    redUpper = np.array([lmax, amax, bmax])

    frame_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    red = cv2.inRange(frame_lab, redLower, redUpper)
    red = cv2.erode(red, None, iterations=8)
    contours_ball, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # red = cv2.dilate(red,None,iterations=10)

    res = np.zeros_like(img)
    if len(contours_ball) > 0:
        contours_ball.sort(key=lambda x: len(x))
        cnt = contours_ball[-1]
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if y > 100:
            ball_x, ball_y = x, y
            last_ball_x = x
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(res, center, radius, (0, 0, 255), -1)
    cv2.imshow("c", res)
    #print("ball_x=%s \n ball_y = %s \n ball_r = %s" % (ball_x, ball_y, radius))
    cv2.imshow('red in range', red)
cv2.destroyAllWindows()
'''

if __name__=='__main__':
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        if cv2.waitKey(1)== 27:
            break
        ret, img = cap.read()
        lab_debug(img)

    cap.release()
    cv2.destroyAllWindows()
'''