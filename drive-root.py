#!/usr/bin/env python3

from pyroot import Root
import time

command = ""
try:
    robot = Root()
    robot.wait_for_connect()

    print("Press letter (f,b,l,r) to drive robot (t) to turn, (s) to stop, (u or d) raise pen up or down, (z) to get sensor states, (i) for sniff, (q) to quit")
    while command != "q" and robot.is_running():
        command = input('> ') # wait for keyboard input
        if command == "f":
            print ("Drive forward")
            robot.set_motor_speeds(100, 100)
        if command == "b":
            print ("Drive backwards")
            robot.set_motor_speeds(-100, -100)
        if command == "r":
            print ("Drive right")
            robot.set_motor_speeds(100, 0)
        if command == "l":
            print ("Drive left")
            robot.set_motor_speeds(0, 100)
        if command == "s":
            print ("Stop")
            robot.set_motor_speeds(0, 0)
        if command == "u":
            print ("Pen up")
            robot.set_marker_eraser_pos(robot.marker_up_eraser_up)
        if command == "d":
            print ("Pen down")
            robot.set_marker_eraser_pos(robot.marker_down_eraser_up)
        if command == "t":
            rate = int(input('Enter turn rate (up to +-90): '))
            print ("Turning ", rate)
            if rate >= 0:
                left = rate
                right = 0
            else:
                left = 0
                right = -1*rate
            robot.set_motor_speeds(left, right)
        if command == "z":
            for s, v in robot.sensor.items():
                print(s, v)
        if command == 'i':
            robot.set_sniff_mode(not robot.get_sniff_mode())
        if command == '`':
            robot.get_versions(robot.main_board)
            robot.get_versions(robot.color_board)

except KeyboardInterrupt:
    pass

print("Quitting")
robot.disconnect()
