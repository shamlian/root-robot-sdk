# Polling line follower:

from wrapi.py.devices.Root.Root_v1_0 import *


def camera1dDidChange(source, data):
    if clRed in robot.camera1d.colors:
        robot.stop()
        return

    # Slices the colors array in 3 areas:
    colorsLeft = robot.camera1d.colors[0:10]
    #colorsCenter = robot.camera1d.colors[10:22]
    colorsRight = robot.camera1d.colors[22:32]
    # print colorsLeft, colorsCenter, colorsRight

    if clBlack in colorsLeft:
        robot.rotate(-90)
    elif clBlack in colorsRight:
        robot.rotate(90)

    robot.speed(fwSpeed, fwSpeed)


fwSpeed = 50
robot = Root_v1_0()
robot.camera1d.dataDidChange = camera1dDidChange

robot.speed(50, 50)