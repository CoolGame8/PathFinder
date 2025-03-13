
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

#constants:
ROTATING_SPEED = 200 # mm/s
MAX_SPEED = 200 # mm/s
MAX_TIME_TO_ROTATE = 3 # seconds

class Node():

    def __init__(self, x, y, rotation, neighbors):
        self.x = x
        self.y = y
        self.rotation = rotation
        self.neighbors = neighbors

    def add_neighbor(self, neighbor, cost):
        self.neighbors.append((neighbor, cost)) 
    
    def print_coords(self):
        return f"({self.x}, {self.y}, {self.rotation})"
    
    def __str__(self):
        return f"Node({self.x}, {self.y}, {self.rotation}, {self.neighbors})"

class NodesHelpers():

    def heuristic(node1, node2):
        return ((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2) ** 0.5

    def reconstruct_path(came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        return path[::-1]

    def a_star(start, end, allowed_nodes):
        open_set = [start]
        came_from = {}
        cost_from_start = {node: float('inf') for node in allowed_nodes}
        estimated_total_cost = {node: float('inf') for node in allowed_nodes}
        cost_from_start[start], estimated_total_cost[start] = 0, NodesHelpers.heuristic(start, end)
        
        while open_set:
            open_set.sort(key=lambda node: estimated_total_cost[node])
            current = open_set.pop(0)
            
            if current == end:
                return NodesHelpers.reconstruct_path(came_from, current)
            
            for neighbor, cost in current.neighbors:
                if neighbor not in allowed_nodes:
                    continue
                tentative_cost = cost_from_start[current] + cost
                if tentative_cost < cost_from_start[neighbor]:
                    came_from[neighbor] = current
                    cost_from_start[neighbor] = tentative_cost
                    estimated_total_cost[neighbor] = cost_from_start[neighbor] + NodesHelpers.heuristic(neighbor, end)
                    if neighbor not in open_set:
                        open_set.append(neighbor)
        
        return None

class PDController:
    def __init__(self, kp, kd, setpoint):
        """
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param setpoint: Desired target position
        """
        self.kp = kp
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
    
    def calculate(self, current_position, dt):
        """
        Compute the control output based on current position.
        :param current_position: The current position of the system
        :param dt: Time step (delta time between updates)
        :return: Control output
        """
        error = self.setpoint - current_position
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = self.kp * error + self.kd * derivative
        self.previous_error = error
        
        return output


class DriveHelpers():

    def get_angle_to_node(starting_node, ending_node):
        x = ending_node.x - starting_node.x
        y = ending_node.y - starting_node.y
        angle = umath.atan2(y, x)
        return angle
        
    def turn_to_angle(angle, speed=ROTATING_SPEED, max_time=MAX_TIME_TO_ROTATE):
        """Turns to a specified absolute gyro angle"""

        timer = StopWatch()
        timer.reset()

        distance = angle - hub.imu.heading()
        robot_acceleration = chassis.settings()[3]
        chassis.settings(turn_rate=speed)
        chassis.turn(distance)

        while (timer.time()) < (max_time * 1000) and angle - hub.imu.heading() > 1:
            pass
        chassis.settings(turn_rate=robot_acceleration)

    def drive_in_straight_line(distance, speed=MAX_SPEED):
        """Drives in a straight line for a specified distance"""

        robot_acceleration = chassis.settings()[2]
        chassis.settings(straight_speed=speed)
        chassis.straight(distance)
        chassis.reset()

        while chassis.distance() < distance:
            pass
        chassis.brake()
        chassis.settings(straight_speed=robot_acceleration)

    def follow_nodes(nodes, starting_node, linear_speed=MAX_SPEED, angular_speed=ROTATING_SPEED):
        last_node = starting_node
        for node in nodes:
            print("current node", node)
            has_angle =  node.rotation is not None
            if has_angle:
                angle = DriveHelpers.get_angle_to_node(last_node, node)
                DriveHelpers.turn_to_angle(angle)
            DriveHelpers.drive_in_straight_line(linear_speed if has_angle else -linear_speed)
            last_node = node
            

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

nodes : list[Node] = []

nodes.append(Node(0, 0, 0, []))
nodes.append(Node(10, 10, 0, []))
nodes.append(Node(200, 0, 0, []))
nodes.append(Node(300, 0, 0, []))

nodes[0].add_neighbor(nodes[1], 1)
nodes[1].add_neighbor(nodes[0], 1)
nodes[2].add_neighbor(nodes[1], 1)
nodes[1].add_neighbor(nodes[3], 1)
nodes[3].add_neighbor(nodes[2], 1)

t_nodes = NodesHelpers.a_star(nodes[0], nodes[3], nodes)
for n in t_nodes:
    print(n)

DriveHelpers.follow_nodes(t_nodes,nodes[0])