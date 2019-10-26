'''
Filename: cam_test.py
Description: Playing around with preprocessing methods to get pyzbar.decode() working.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''


import cv2
from pyzbar.pyzbar import decode
import numpy as np
import imutils



def detect_qr(img, n=2):
     # Modified from https://www.pyimagesearch.com/2014/12/15/real-time-barcode-detection-video-python-opencv/
     # Not actually helpful...

    # Blur
    fil = cv2.bilateralFilter(img,25,100,100)
    # normalize lighting
    norm = cv2.normalize(fil,fil,0,255,cv2.NORM_MINMAX)

    # Up contrast
    # gray = cv2.addWeighted(gray, 3, np.zeros(gray.shape, gray.dtype), 0, 0)
 
    # compute the Scharr gradient magnitude representation of the images
    # in both the x and y direction using OpenCV 2.4
    ddepth = cv2.cv.CV_32F if imutils.is_cv2() else cv2.CV_32F
    gradX = cv2.Sobel(norm, ddepth=ddepth, dx=1, dy=0, ksize=-1)
    gradY = cv2.Sobel(norm, ddepth=ddepth, dx=0, dy=1, ksize=-1)
 
    # subtract the y-gradient from the x-gradient
    gradient = cv2.subtract(gradX, gradY)
    gradient = cv2.convertScaleAbs(gradient)

    # blur and threshold the image
    # blurred = cv2.blur(gradient, (10, 10))
    blurred = gradient
    # blurred = cv2.addWeighted(blurred, 3, np.zeros(blurred.shape, blurred.dtype), 0, 0)
    # blurred = gradient
    _, thresh = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)
    # _, thresh = cv2.threshold(blurred,0,255,cv2.THRESH_OTSU)

    # construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20,20))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # perform a series of erosions and dilations
    closed = cv2.erode(closed, None, iterations = 4)
    closed = cv2.dilate(closed, None, iterations = 4)


    # find the contours in the thresholded image, then sort the contours
    # by their area, keeping only the largest one
    cnts = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = sorted(cnts, key = cv2.contourArea, reverse = True)
     
    if n > len(c):
        n = len(c)

    # compute the rotated bounding box of the largest n contours
    box = []
    for cc in c:
        if cv2.contourArea(cc) > 200:
            rect = cv2.minAreaRect(cc)
            bx = cv2.boxPoints(rect) # cv2.cv.BoxPoints(rect) if imutils.is_cv2() else cv2.boxPoints(rect)
            box.append(np.int0(bx))

    return box



def show_cam(camera=0, bw=False, mirrorlr=False, mirrorud=False, hsv=False, detect_region=False):
    cam = cv2.VideoCapture(camera)
    try:
        while True:
            img_available, img = cam.read()


            if img_available==True:
                # convert the image to grayscale
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                img = cv2.normalize(img,img,0,255,cv2.NORM_MINMAX)
                img = cv2.addWeighted( img, 1.5, np.zeros(img.shape, img.dtype), 0, 0)

                
                if detect_region:
                    box = detect_qr(img)
                    # draw a bounding box arounded the detected QR code 
                    # cv2.drawContours(img, [box], -1, (0, 255, 0), 3)

                    # Set area outside region of interest to white
                    mask = np.zeros_like(img) # Create mask where white is what we want, black otherwise
                    for bx in box:
                        cv2.fillConvexPoly(mask, np.array(bx), 255)
                    out = 255*np.ones_like(img)
                    out[mask == 255] = img[mask == 255]
                    img = out


                if hsv:
                    img = cv2.normalize(img,img,0,255,cv2.NORM_MINMAX)
                    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


                    # define range of bw color in HSV
                    sat_limit = 40
                    lower_bw = np.array([0,0,0])
                    upper_bw = np.array([255,sat_limit,255])

                    # Threshold the HSV image to get only white colors
                    mask = cv2.inRange(img_hsv, lower_bw, upper_bw)
                    # Bitwise-AND mask and original image
                    img = cv2.bitwise_and(img,img, mask= mask)
                
                # Cast to BW image
                if bw:
                    # Up contrast
                    img = cv2.addWeighted(img, 3, np.zeros(img.shape, img.dtype), 0, 0)

                    # blur noise
                    img = cv2.bilateralFilter(img,5,75,75)

                    
                    # blur = cv2.GaussianBlur(img,(5,5),0)
                    # _, img = cv2.threshold(img,0,255,cv2.THRESH_OTSU)
                    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
                    # _, img = cv2.threshold(img, img.mean()/2, 255, cv2.THRESH_TOZERO)
                    # _, img = cv2.threshold(img, 200, , cv2.THRESH_TOZERO_INV)



                decoded = decode(img)
                if len(decoded) > 0:
                    print(decoded)

                # Flip image if necessary
                if mirrorlr:
                    img = cv2.flip(img,1)
                if mirrorud:
                    img = cv2.flip(img,0)

                cv2.imshow('my cam', img)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()

def main():
    show_cam(camera=1)

if __name__ == '__main__':
    main()
