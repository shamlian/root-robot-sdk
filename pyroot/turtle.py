# External Dependencies
import turtle

# Internal Dependencies
from .root import Root

class Turtle(Root):
    """Simulate a Root robot using the Python turtle.
    
    Currently only supports a subset of methods in the Root class.
    """
    robot = None

    def __init__(self, name = None):
        self.robot = turtle.Turtle()
        self.robot.setheading(90)
        self.robot.penup()

    def wait_for_connect(self):
        pass

    def is_running(self):
        return not self.stop_project_flag.is_set()

    def disconnect(self):
        self.stop_project_flag.set()

    def transmissions_pending(self):
        return False

    def drive_distance(self, distance):
        self.robot.forward(distance)

    def rotate_angle(self, angle):
        self.robot.right(angle/10)

    def set_marker_eraser_pos(self, pos):
        if pos == self.marker_up_eraser_up:
            self.robot.penup()
        else:
            self.robot.pendown()
