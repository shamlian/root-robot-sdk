#!/usr/bin/env python3

from pyroot import Root
import time

command = ""
try:
    robot = Root()
    robot.wait_for_connect()

    print("Press letter (f,b,l,r) to drive robot (t) to turn, (s) to stop, (u or d) raise pen up or down, (m {val}) to move, (a{val}) to rotate, (z) to get robot states, (i) for sniff, (q) to quit")
    while command != "q" and robot.is_running():
        command = input('> ') # wait for keyboard input
        if command == "f":
            print("Drive forward")
            robot.set_motor_speeds(100, 100)
        if command == "b":
            print("Drive backwards")
            robot.set_motor_speeds(-100, -100)
        if command == "r":
            print("Drive right")
            robot.set_motor_speeds(100, 0)
        if command == "l":
            print("Drive left")
            robot.set_motor_speeds(0, 100)
        if command == "s":
            print("Stop")
            robot.set_motor_speeds(0, 0)
        if command == "u":
            print("Pen up")
            robot.set_marker_eraser_pos(robot.marker_up_eraser_up)
        if command == "d":
            print("Pen down")
            robot.set_marker_eraser_pos(robot.marker_down_eraser_up)
        if command[0] == "m":
            try:
                dist = int(command.split()[1])
                print("Moving", dist, 'mm')
                robot.drive_distance(dist)
            except:
                print("Bad command")
        if command[0] == "a":
            try:
                angle = int(command.split()[1])
                print("Rotating", angle, 'decidegrees')
                robot.rotate_angle(angle)
            except:
                print("Bad command")
        if command == "z":
            for s, v in robot.state.items():
                print(s, v)
        if command == 'i':
            robot.set_sniff_mode(not robot.get_sniff_mode())
        if command == '`':
            robot.get_versions(robot.main_board)
            robot.get_versions(robot.color_board)
            robot.get_battery_level()

except KeyboardInterrupt:
    pass

print("Quitting")
robot.disconnect()
