__author__ = 'Danny'

import time
import pygame
import serial

ser = serial.Serial('COM4', 9600)  # open first serial port

pygame.init()
pygame.joystick.init()
done = False

joystick = pygame.joystick.Joystick(0)
joystick.init()

def getJoyVal( axis ):
    "this function returns the value of a joystick, given an axis and a joystick number"
    if abs(joystick.get_axis(axis))>0.1:
        Value = 255-(int(round(127*joystick.get_axis(axis)))+127)
    else:
        Value = 127
    #map (-1 to 1) to (0.10 to 0.20) for PWM
    #pwm = chr(Value)
    #fpwm = '{:4.4}'.format(pwm)     #for sending direct pwm value
    return Value

def getButtonVal(button):
    x = joystick.get_button(button)
    return x

# def camSelect():
#         counter = 0
#         if(getButtonValue(6))
#             if counter = 0
#                 cam_select = 0
#             if counter = 1
#                 cam_select = 127
#             if counter = 2
#                 cam_select = 255
#     return cam_select

while 1:
    packet = bytearray()
    packet.append(60)                 # 0: <
    #packet.append(getJoyVal(3))       # 1: joystick 3
    #packet.append(getJoyVal(1))       # 2: joystick 1
#   packet.append(camSelect())       #
    packet.append(32)
    packet.append(72)
    packet.append(101)
    packet.append(108)
    packet.append(108)
    packet.append(111)
    packet.append(32)
    packet.append(119)
    packet.append(111)
    packet.append(114)
    packet.append(108)
    packet.append(100)
    packet.append(33)
    packet.append(62)                 # 3: >
    pygame.event.get()
#    x = getButtonVal(6)

    print (packet)
    time.sleep(.1)
    ser.write(packet)