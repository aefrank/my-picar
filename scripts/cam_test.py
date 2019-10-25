import cv2
from pyzbar.pyzbar import decode

def show_cam(bw=False, mirrorlr=False, mirrorud=False, camera=0):
    cam = cv2.VideoCapture(camera)
    try:
        while True:
            img_available, img = cam.read()

            if img_available==True:
                

                # Cast to BW image
                if bw:
                    # Grayscale
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
                    # print(img.shape)
                    # blur image
                    img = cv2.bilateralFilter(img,15,100,100)
                    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)


                    # lighting = img.mean()
                    # th = (lighting)
                    # # img = img + (127-th)
                    # # print(th)
                    # # _, img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)

                    # maskw = img > 200#(th+(255-th)/2)
                    # maskb = img < 50 #(th-th/2)
                    # # img[maskb] = 0
                    # # img[maskw] = 255
                    # maskg = 1 - (maskw+maskb)

                    # maskw = maskw.astype('uint8')
                    # maskb = maskb.astype('uint8')
                    # maskg = maskg.astype('uint8')

                    # imgw = cv2.bitwise_and(img, maskw)
                    # imgb = 1 - cv2.bitwise_and(img, maskb)
                    # # imgg = cv2.bitwise_and(img, maskg)

                    # img = cv2.bitwise_or(imgw,imgb)
                    # # img = cv2.bitwise_or(img,imgg)
                    # # img[maskg] = 127
                    # # img = 255*maskw + 0*maskb + 127*( (1-maskw)*(1-maskb) )
                    # # print(img.shape)


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
    show_cam(camera=1, bw=True, mirrorlr=False)

if __name__ == '__main__':
    main()
