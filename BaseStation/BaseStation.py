__author__ = 'Danny'

import time
import pygame
import serial

pygame.init()
pygame.joystick.init()
done = False
counter = 0
lastButton = 0

joystickWheels = pygame.joystick.Joystick(0)
joystickWheels.init()
joystickArm = pygame.joystick.Joystick(0)
joystickArm.init()


def getJoyVal(joystick, axis, deadband):
    # type: (object, object) -> object
    "this function returns the value of a joystick, given an axis and a joystick number"
    if abs(joystick.get_axis(axis)) > deadband:
        Value = 255 - (int(round(127 * joystick.get_axis(axis))) + 127)
    else:
        Value = 127
    # map (-1 to 1) to (0.10 to 0.20) for PWM
    # pwm = chr(Value)
    # fpwm = '{:4.4}'.format(pwm)     #for sending direct pwm value
    return Value


def getHat(axis, joystick, number):
    if axis == 'x':
        if str(joystick.get_hat(number))[1] == '1':
            value = 64
        elif str(joystick.get_hat(number))[1] == '-':
            value = 192
        else:
            value = 127
    if axis == 'y':
        if str(joystick.get_hat(number))[4] == '1':
            value = 192
        elif str(joystick.get_hat(number))[4] == '-':
            value = 64
        else:
            value = 127
    return value


def getState(button):
    global counter
    global lastButton
    if (button == 1) and (lastButton == 0):
        lastButton = 1
        state = counter
        counter += 1
        if counter >= 3:
            counter = 0
        return state
    if button == 0:
        lastButton = 0


ser = serial.Serial('COM4', 9600)  # open first serial port
# print ser.portstr       # check which port was really used
# ser.write("<@@@>")      # write a string
# ser.close()





while 1:
    pygame.event.get()
    my_bytes = bytearray()

    # if(joystickArm.get_button(1)):
    #     armActuator

    # if getState == 0:
    my_bytes.append(60)
    my_bytes.append(getJoyVal(joystickWheels, 1, 0.05))  # 1 left wheels
    my_bytes.append(getJoyVal(joystickWheels, 3, 0.05))  # 2 right wheels
    my_bytes.append(getHat('x', joystickWheels, 0))  # 3 cam pan, wheels hat
    my_bytes.append(getHat('y', joystickWheels, 0))  # 4 cam tilt, wheels hat
    if (joystickWheels.get_button(7) == 1):
        my_bytes.append(64)  # 5 cam select, select button
    if (joystickWheels.get_button(7) == 0):
        my_bytes.append(192)  # 5 cam select, select button
    ###########Arm Actuators##########
    if joystickArm.get_button(0) == 1:
        my_bytes.append(getJoyVal(joystickArm, 1, 0.05))  # 6 upper arm speed - axis 3
        my_bytes.append(127)  # 7 lower arm speed
    if joystickArm.get_button(0) == 0:
        my_bytes.append(127)  # 6 upper arm speed - axis 3
        my_bytes.append(255 - getJoyVal(joystickArm, 1, 0.05))  # 7 lower arm speed
    ##################################
    ##########Base Rotation###########
    my_bytes.append(255 - getJoyVal(joystickArm, 0, 0.15))  # 8 base rotation
    ##################################
    ##########Wrist Rotation##########
    if joystickArm.get_button(4) == 1:
        my_bytes.append(30)  # 9 wrist rot - x and y
    elif joystickArm.get_button(5) == 1:
        my_bytes.append(220)
    else:
        my_bytes.append(127)  # 9 wrist rot - x and y
    my_bytes.append(255 - getJoyVal(joystickArm, 4, 0.15))  # 10 wrist yaw - axis 4?
    #################################
    ##########Wrist Pitch###########
    # if(getJoyVal(joystickArm, 3, 0.15) < 45):
    #     my_bytes.append(45)
    # if(getJoyVal(joystickArm, 3, 0.15) >= 170):
    #     my_bytes.append(85)
    # else:
    #     my_bytes.append(255-getJoyVal(joystickArm, 3, 0.05))  # 11 wrist pitch - axis 1
    my_bytes.append(getHat('y', joystickArm, 0))  # 11 wrist pitch - arm hat y
    ###############################
    my_bytes.append(getJoyVal(joystickArm, 2, 0.15))  # 12 wrist claw - axis 2?
    if (joystickArm.get_button(6) == 1):
        my_bytes.append(115)  # 13 soil sensor request
    else:
        my_bytes.append(127)
    my_bytes.append(127)  # 14 gps request
    my_bytes.append(62)

    # bytesToRead = ser.inWaiting()
    # print ser.read(bytesToRead)

    # z = '<'+chr(y1)+chr(y2)+'a'+'>'
    # z = bytes(0x3c)+bytes(0x40)+bytes(0x56)+bytes(0x55)+bytes(0x3E)
    # z=[float(x1),float(y1),float(x2),float(y2)]

    # ser.write(str(z))
    # print getHat('y',joystickWheels,0)
    print(my_bytes)

    ser.write(my_bytes)
    # bytesToRead = ser.inWaiting()
    # print ser.read(bytesToRead)
    time.sleep(.05)

    # ser.close()
