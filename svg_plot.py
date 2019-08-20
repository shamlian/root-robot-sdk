#!/usr/bin/env python3

from pyroot import Root

#BUG: treats all segments as lines
# helpful links:
# https://pypi.org/project/svgpathtools/
# https://nerd.mmccoo.com/2018/01/11/understanding-and-parsing-svg-paths/
# https://www.limosa.io/post/pen-plotter/

import svgpathtools
import numpy
import time
import argparse

parser = argparse.ArgumentParser(description='Plot an SVG path with Root.')
parser.add_argument('-s', '--scale', type=float, default=1, help='Scalar for points')
parser.add_argument('filename', type=str, help='SVG with path to plot')

args = parser.parse_args()

try:
    paths, attributes = svgpathtools.svg2paths(args.filename)
except FileNotFoundError as e:
    print(e)
    exit(1)

def invert_y(c): # I feel like there should be a better way
    return numpy.real(c) - numpy.imag(c)*1j

try:
    robot = Root()
    robot.wait_for_connect()
    time.sleep(1) 

    for path in paths:
        for element in path:
            start = invert_y(element.start) * args.scale
            end   = invert_y(element.end  ) * args.scale
            if robot.last_coord != start:
                robot.set_marker_eraser_pos(robot.marker_up_eraser_up)
                robot.drive_complex(start)
                robot.set_marker_eraser_pos(robot.marker_down_eraser_up)
            robot.drive_complex(end)

    robot.set_marker_eraser_pos(robot.marker_up_eraser_up)

    while(not robot.tx_q.empty()):
        time.sleep(1)  # block until queue empty
except KeyboardInterrupt:
    pass

robot.disconnect()
