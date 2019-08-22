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
parser.add_argument('-a', '--approximate', type=int, default=5, help='Number of lines to use to approximate curves')
parser.add_argument('-l', '--min_length', type=float, default=10, help='Minimum curve length for approximation')
parser.add_argument('-e', '--epsilon', type=float, default=0.5, help='Distance between endpoints to consider "close enough" to not lift the pen')
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

    segments = numpy.linspace(0, 1, args.approximate + 1)

    for path in paths:
        for element in path:
            if type(element) == svgpathtools.path.Line or element.length()*args.scale < args.min_length:
                line_list = [element]
            else: # adapted from svgpathtools issue #61
                pts = [element.point(t) for t in segments]
                line_list = [svgpathtools.Line(pts[i-1], pts[i]) for i in range(1, len(pts))]

            for line in line_list:
                start = invert_y(line.start) * args.scale
                end   = invert_y(line.end  ) * args.scale
                # if the robot's last known position isn't close enough to the start, lift the pen
                if numpy.linalg.norm(robot.last_coord - start) > args.epsilon:
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
