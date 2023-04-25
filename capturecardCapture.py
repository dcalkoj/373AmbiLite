import numpy as np
import cv2 as cv
import serial

debug = False

cap = cv.VideoCapture(1)
ser = serial.Serial("/dev/ttyS0",921600, parity=serial.PARITY_NONE,stopbits=1, bytesize=serial.EIGHTBITS,timeout=100)

'''cap.set(set)
cap.set(4, 720)'''

#how many  LEDs do we have on each side of TV
xsteps = 37
ysteps = 21

#Relative positioning of TV border in Pi Camer Image
top = 0
bottom = 21
left = 0
right = 37
borderWidth = right - left
borderHeight = bottom - top

hmod=4
wmod=0
top = top + hmod
bottom = bottom-hmod
right = right+wmod
left=left-wmod

#distance between each sampled pixel
xstepLength = borderWidth/xsteps
ystepLength = borderHeight/ysteps

if debug:
    img= np.zeros([ysteps, xsteps, 3])

if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
   #top border
	if ser.inWaiting()>0:
		wanted = ser.read(1)
		ser.reset_input_buffer()
		ser.write([101])
        
        if debug:
            print('ACK. I read a ',  (wanted))

		ret, frame = cap.read()
		if not ret:
				print("cant reveice")
				break
		height, width = frame.shape[:2]
		img_temp = cv.resize(frame, (37, 21), interpolation=cv.INTER_NEAREST)
        
        if debug:
            cv.startWindowThread()
            cv.imshow('original', frame)
            cv.imshow('inerpolated', img_temp)

		t =img_temp[top][0]
		for i in range(1,xsteps):
			t = np.concatenate((t, img_temp[top][i*xstepLength]),axis=0)
			#print('Top: ', t)

		t = np.concatenate( (t, img_temp[top][right-1]),axis=0)
		for i in range(1,ysteps):
			t=np.concatenate((t,img_temp[i*ystepLength][right-1]),axis=0)
			#print('Right: ', t)

		t = np.concatenate( (t, img_temp[bottom-1][right-1]),axis=0)
		for i in range(xsteps-1,0,-1):
			t=np.concatenate((t,img_temp[bottom-1][i*xstepLength]),axis=0)
			#print('Bottom ', t)
			
		t = np.concatenate( (t, img_temp[bottom-1][left]),axis=0)
		for i in range(ysteps-1,0,-1):
			t=np.concatenate((t,img_temp[i*ystepLength][left]),axis=0)
			t=t.tolist()
			#print('Left ', t)
			
		if debug:
            print(t)
            for i in range(xsteps):
                    img[0, i, 0] = t[i*3 +0]
                    img[0, i, 1] = t[i*3 +1]
                    img[0, i, 2] = t[i*3 +2]
            for i in range(ysteps):
                    img[i,xsteps-1, 0] = t[(xsteps+i)*3 +0]
                    img[i,xsteps-1, 1] = t[(xsteps+i)*3 +1]
                    img[i,xsteps-1, 2] = t[(xsteps+i)*3 +2]
            for i in range(0, xsteps):
                    img[ysteps-1, xsteps-1-i, 0] = t[(xsteps+ysteps+i)*3 +0]
                    img[ysteps-1, xsteps-1-i, 1] = t[(xsteps+ysteps+i)*3 +1]
                    img[ysteps-1, xsteps-1-i, 2] = t[(xsteps+ysteps+i)*3 +2]
            for i in range(0,ysteps, 1):
                    img[ysteps-i-1, 0, 0] = t[(2*xsteps+ysteps+i)*3 +0]
                    img[ysteps-i-1, 0, 1] = t[(2*xsteps+ysteps+i)*3 +1]
                    img[ysteps-i-1, 0, 2] = t[(2*xsteps+ysteps+i)*3 +2]
            cv.imshow('finalOutput', img/255.0)

		ser.write(t)
        
cap.release()
cv.destroyAllWindows()