import sensor, image, time
import math
import time
from pyb import Pin, Timer
import pyb
from pyb import UART

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QQVGA (160x120)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()                # Create a clock object to track the FPS.
uart = UART(1, 115200, timeout_char=1000)

tim1 = Timer(4, freq = 300) # Servo Frequency in Hz
tim2 = Timer(2, freq = 300) # Motor Frequency in Hz
initialPulseWidth = (15/10000)  * (tim1.source_freq() / (tim1.prescaler()+1))#servo
initialPulseWidth2 = (0/10000)  * (tim2.source_freq() / (tim2.prescaler()+1))#motor
ch1 = tim1.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width = int(initialPulseWidth))#servo
ch2 = tim2.channel(4, Timer.PWM, pin=Pin("P5"), pulse_width = int(initialPulseWidth2))#motor

inA = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE) #set inA to output
inB = Pin('P2', Pin.OUT_PP, Pin.PULL_NONE) #set inB to output
enA = Pin('P3', Pin.OUT_PP, Pin.PULL_NONE) #set enA to output
enB = Pin('P4', Pin.OUT_PP, Pin.PULL_NONE) #set enB to output

def forward(): #motor control forward
    inA.high()
    inB.low()
    enA.high()
    enB.high()

def reverse(): #motor control reverse
    inA.low()
    inB.high()
    enA.high()
    enB.high()

def brakeVCC(): #motor control break to VCC
    inA.high()
    inB.high()
    enA.high()
    enB.high()

def brakeGND(): #motor control break to GND
    inA.low()
    inB.low()
    enA.high()
    enB.high()


#kp 20 kd 30 speed 10 5
#kp 35 kd 15 speed 14 8.5
Kp = 30.0
Kd = 20.0
Ki = 0.0
differ = 0
integr = 0
blobWidth = 0

def dTerm(error):
    errorOut = (error[3]-error[0])+(3*error[2])-(3*error[1])
    return errorOut


thresholds = (235, 255)
ROIS = [(0, 30, 160, 10)]
finishROIS = [(0, 90, 160, 10)]
roiX = 1
c = '0'
xIndex = 0
xdiff = 0
errorBuff = [0,0,0,0]
widthBuff = [0,0,0,0]
brakeFlag = 0
noBrake = 0
straightCount = 0

forward()

while(1):
    if(uart.any()):
        c = uart.readchar() # save characters

    ch2.pulse_width(int(0))

    while(1):
        #if(uart.any()): #check if UART has characters
            #c = uart.readchar() # save characters
            #print(c)

        clock.tick()                    # Update the FPS clock.
        img = sensor.snapshot()         # Take a picture and return the image.

        #for rfinish in finishROIS :
             #finishblobs = img.find_blobs([thresholds], roi=rfinish, pixels_threshold=100, area_threshold=100, merge=True)
             #if len(finishblobs) == 3:
                 #while(1):
                    #brakeVCC()

        for r in ROIS :
            blobs = img.find_blobs([thresholds], roi=r, pixels_threshold=100, area_threshold=100, merge=True)
            if blobs:
                center_blob = min(blobs, key=lambda b: abs(80 - b.cx()))
                blobWidth = center_blob.w()
                img.draw_rectangle(center_blob.rect(), color=(0,0,0))  #draw rectangle around blob
                img.draw_cross(center_blob.cx(), center_blob.cy(),color=(0,0,0))  #draw cross in center of blob
                roiX = center_blob.cx()  #fill center x postions

                xdiff = 80.0 - roiX

                errorBuff.append(xdiff)		#append and pop buffer
                errorBuff.pop(0)

                widthBuff.append(blobWidth)
                widthBuff.pop(0)

                differ = dTerm(errorBuff)
                integr = sum(errorBuff)

#MOTOR CONTROL
        if (xdiff > -80.0 and xdiff < -10.0): #1 or a
            desiredPulseWidthMotor = 8.0
        elif(xdiff > -10.0 and xdiff < 10.0): #5 or g
            desiredPulseWidthMotor = 12.0
            straightCount += 1
        elif(xdiff > 10.0 and xdiff < 80.0): #9 or l
            desiredPulseWidthMotor = 8.0
        else:
            desiredPulseWidthMotor = 1
            continue

#BRAKE

        if((abs(sum(errorBuff))>90)and(straightCount>=25)and(brakeFlag<=0)and(sum(widthBuff)<100)):
            reverse()
            brakeFlag = 0.2*straightCount
            straightCount = 0
        if (brakeFlag > 0):
            desiredPulseWidthMotor=4.0
            brakeFlag-=1
        else:
            forward()


#PWM SIGNALS
        #4968-7992
        #actualPulseWidthServo = 6480+(Kp*xdiff) + (Kd*differ) + (Ki*integr)

        actualPulseWidthServo = 6880+(Kp*xdiff) + (Kd*differ) + (Ki*integr)

        if((actualPulseWidthServo<4968) or (actualPulseWidthServo>7992)):
            integr = 0
        ch1.pulse_width(int(actualPulseWidthServo))

        desiredPulseWidthMotor = desiredPulseWidthMotor / 10000
        actualPulseWidthMotor = desiredPulseWidthMotor * (tim2.source_freq() / (tim2.prescaler()+1))
        ch2.pulse_width(int(actualPulseWidthMotor))
