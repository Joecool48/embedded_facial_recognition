from picamera.array import PiRGBArray
from picamera import PiCamera
from Adafruit_PWM_Servo_Driver import PWM
import os, sys
from operator import itemgetter
import serial
import time
import cv2
import random
#The address of the horizontal and vertical motors
horizontal_motor = 0
vertical_motor = 1
#Tolerance of the what is a centered image
tolerance_factor = 8
#Focal length of the raspberry pi
focal_length = 2.7
pwm = PWM(0x40)
pwm.setPWM(1,0,575)
time.sleep(5)
serial = serial.Serial("/dev/ttyUSB0", 9600, timeout = 5)
servomin = 150  # Min pulse length out of 4096
servomax = 600  # Max pulse length out of 4096
vertical_center = servomin + (servomax - servomin) // 2
#Get serial input from arduino
distance = 0
#The servo x and y gains
gainx = 10
gainy = 5
def get_distance():
    line = serial.readline()
    if len(line) == 0:
        print("No data available")
        return 0
    return line
res = (640,480)
def setServoPulse(channel, pulse):
    pulseLength = 1000000                   # 1,000,000 us per second
    pulseLength /= 60                       # 60 Hz
    print "%d us per period" % pulseLength
    pulseLength /= 4096                     # 12 bits of resolution
    print "%d us per bit" % pulseLength
    pulse *= 1000
    pulse /= pulseLength
    pwm.setPWM(channel, 0, pulse)
pwm.setPWMFreq(60)
#The servo positions with respect to pulse for horizontal and vertical directions
pwmx = servomin
pwmy = 575
pwm.setPWM(horizontal_motor,0,pwmx)
pwm.setPWM(vertical_motor,0,pwmy)
#Distance in meters, height of object in pixels
def height_object(distance, height, VFOV = .85):
    global res
    return (distance * height  * VFOV) / res[1]
#Distance in meters ,height of object in pixels
def width_object(distance, width, HFOV = 1.09):
    global res
    return (distance * width * HFOV) / res[0]

cascPath = 'haarcascade_frontalface_default.xml'
faceCascade = cv2.CascadeClassifier(cascPath)
camera = PiCamera()
camera.resolution = (res)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(res))
time.sleep(2)
distancecount = 0
goright = False
goup = False
framecount = 0
currentface = ()
for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port=True):
    temp = get_distance()
    if temp == 0:
        distancecount += 1
    elif temp != 0:
        distancecount = 0
        distance = temp
    if distancecount > 3:
        distance = 0
    print(distance)
    # Capture frame-by-frame
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(gray,1.3,4)
    #If it cant find any faces, begin searching for them
    if (framecount > 8 or distancecount >= 5) and (distance > 200 or distance == 0) :
        #search in some fashion
        framecount += 1
        #tol = (servomax - servomin) / 10
        """if tol + pwmx >= servomax:
            pwmx -= gainx
            pwm.setPWM(horizontal_motor,0,int(pwmx))
        elif pwmx - tol <= servomin:
            pwmx += gainx
            pwm.setPWM(horizontal_motor,0,int(pwmx))
        else:
            rand = random.randint(0, 2 * gainx)
            pwmx += rand
            pwm.setPWM(horizontal_motor,0, int(pwmx))
        if tol / 2 + pwmy >= servomax / 2:
            pwmy -= gainy
            pwm.setPWM(vertical_motor,0,int(pwmy))
        elif pwmy - tol / 2 <= servomin:
            pwmy += gainy
            pwm.setPWM(vertical_motor,0,int(pwmy))
        else:
            rand = random.randint(0, 2 * gainy)
            pwmy += rand
            pwm.setPWM(vertical_motor,0, int(pwmy))"""
        print("Searching") 
        #pwm.setPWM(vertical_motor,0, int(vertical_center * 3 / 2))
        if not goup:
            if gainx + pwmx > servomax:
                goright = True
                pwmx -= gainx
                pwm.setPWM(horizontal_motor,0,pwmx)
            elif goright:
                pwmx -= gainx
                pwm.setPWM(horizontal_motor,0,pwmx)
            elif pwmx - gainx < servomin:
                goright = False
                pwmx += gainx
                pwm.setPWM(horizontal_motor,0,pwmx)
            else:
                pwmx += gainx
                pwm.setPWM(horizontal_motor,0,pwmx)
        if distance != 0 and len(faces) == 0 and pwmy - gainy >= vertical_center:
            #Look up for a face
            goup = True
            pwmy -= gainy
            pwm.setPWM(vertical_motor,0,pwmy)
    if len(faces) != 0:
        distancecount = 0
        if goup and pwmy <= (5 * vertical_center) / 8:
            #Reset the position
            goup = False
            pwmy = int(servomax - 5 * gainy)
            pwm.setPWM(vertical_motor,0,pwmy)
        elif len(faces) != 0:
            goup = False
            #We found someone
            #Take picture and reset position
            #if len(faces) != 0:
            m = max(faces,key=itemgetter(2))[2]
            frame = 0
            while frame < 10:
                for (x,y,w,h) in faces:
                    if m != w or m != h:
                        continue
                    frame += 1
                    framecount = 0
                    #if m != w or m != h or len(faces) == 0 and framecount >= 4:
                        #continue
                    #currentface = rect(x,y,w,h)
                    print(distance)
                    cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
                    #print("(" + str(width_object(distance = .3, width = w)) + ", " + str(height_object(distance = .3, height = h)) + ")")
                    diffx = (x + (w / 2) - (res[0] / 2))
                    diffy = (y + (h / 2) - (res[1] / 2))
                    if not (x + (w / 2) > res[0] / 2 and abs(diffx) > res[0] / tolerance_factor):
                        print("left")
                        if pwmx + gainx >= servomax:
                            pwm.setPWM(horizontal_motor,0, servomax)
                            pwmx = servomax
                        else:
                            pwmx -= gainx
                            pwm.setPWM(horizontal_motor,0,int(pwmx))
                        #Far left is servomax
                        #move horizontally left
                    elif not (x + (w / 2) < res[0] / 2 and abs(diffx) > res[0] / tolerance_factor):
                        print("right")
                        if pwmx - gainx <= servomin:
                            pwm.setPWM(horizontal_motor,0,servomin)
                            pwmx = servomin
                        else:
                            pwmx += gainx
                            pwm.setPWM(horizontal_motor,0,int(pwmx))
                        #Far right is servomin
                        #move horizontally right
                    if y + (h / 2) > res[1] / 2 and abs(diffy) > res[1] / tolerance_factor:
                        print("up")
                        if pwmy - gainy <= vertical_center:
                            pwm.setPWM(vertical_motor,0,vertical_center)
                            pwmy = vertical_center
                        else:
                            pwmy -= gainy
                            pwm.setPWM(vertical_motor,0,int(pwmy))
                        #Up is servomax / 2 technically
                        #move vertically up
                    if y + (h / 2) < res[1] / 2 and abs(diffy) > res[1] / tolerance_factor:
                        print("down")
                        if pwmy + gainy <= servomax:
                            pwm.setPWM(vertical_motor,0, servomax - 5 * gainy)
                            pwmy = servomax - 5 * gainy
                        else:
                            pwmy -= gainy
                            pwm.setPWM(vertical_motor,0,int(pwmy))
                        #Down is servomin
                        #move vertically down
            
            print("Gotcha")
            camera.capture("image.png")
            pwmy = int((3 * vertical_center) / 2)
            pwm.setPWM(vertical_motor,0,pwmy)
    elif len(faces) == 0:
        framecount += 1
    else:
        framecount = 0
    #if len(faces) != 0:
        #m = max(faces,key=itemgetter(2))[2]
    #if len(faces) == 0 and framecount < 5 and len(currentface) != 0:
        #faces.append(currentface)
    for (x,y,w,h) in faces:
        framecount = 0
        #if m != w or m != h or len(faces) == 0 and framecount >= 4:
            #continue
        #currentface = rect(x,y,w,h)
        print(distance)
        cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
        #print("(" + str(width_object(distance = .3, width = w)) + ", " + str(height_object(distance = .3, height = h)) + ")")
        diffx = (x + (w / 2) - (res[0] / 2))
        diffy = (y + (h / 2) - (res[1] / 2))
        if not (x + (w / 2) > res[0] / 2 and abs(diffx) > res[0] / tolerance_factor):
            print("left")
            if pwmx + gainx >= servomax:
                pwm.setPWM(horizontal_motor,0, servomax)
                pwmx = servomax
            else:
                pwmx -= gainx
                pwm.setPWM(horizontal_motor,0,int(pwmx))
            #Far left is servomax
            #move horizontally left
        elif not (x + (w / 2) < res[0] / 2 and abs(diffx) > res[0] / tolerance_factor):
            print("right")
            if pwmx - gainx <= servomin:
                pwm.setPWM(horizontal_motor,0,servomin)
                pwmx = servomin
            else:
                pwmx += gainx
                pwm.setPWM(horizontal_motor,0,int(pwmx))
            #Far right is servomin
            #move horizontally right
        if y + (h / 2) > res[1] / 2 and abs(diffy) > res[1] / tolerance_factor:
            print("up")
            if pwmy - gainy <= vertical_center:
                pwm.setPWM(vertical_motor,0,vertical_center)
                pwmy = vertical_center
            else:
                pwmy -= gainy
                pwm.setPWM(vertical_motor,0,int(pwmy))
            #Up is servomax / 2 technically
            #move vertically up
        if y + (h / 2) < res[1] / 2 and abs(diffy) > res[1] / tolerance_factor:
            print("down")
            if pwmy + gainy <= servomax:
                pwm.setPWM(vertical_motor,0, servomax - 5 * gainy)
                pwmy = servomax - 5 * gainy
            else:
                pwmy -= gainy
                pwm.setPWM(vertical_motor,0,int(pwmy))
            #Down is servomin
            #move vertically down
    # Display the resulting frame
    # cv2.imshow('frame', image)
    rawCapture.truncate(0)
    if cv2.waitKey(33) == 27:
        break
    

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()