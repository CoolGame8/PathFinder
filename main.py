
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


PI = 3.1415926535898
# Hardware definitions =================================================================
WHEEL_DIAMETER = 62.4 #mm

hub = PrimeHub()
left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)

chassis = DriveBase(
    left_wheel, right_wheel, wheel_diameter=WHEEL_DIAMETER, axle_track=129.4
)
chassis.use_gyro(True)

class Node():

    def __init__(self, x, y, rotation, list_of_nodes):
        self.x = x
        self.y = y
        self.rotation = rotation
        self.list_of_nodes = list_of_nodes
        

nod = Node(1, 2, 0, [])
print(nod.x)
print(nod.y)