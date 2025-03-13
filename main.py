
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
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
nodes.append(Node(1, 0, 0, []))
nodes.append(Node(2, 0, 0, []))
nodes.append(Node(3, 0, 0, []))

nodes[0].add_neighbor(nodes[1], 1)
nodes[1].add_neighbor(nodes[0], 1)
nodes[2].add_neighbor(nodes[1], 1)
nodes[1].add_neighbor(nodes[3], 1)
nodes[3].add_neighbor(nodes[2], 1)

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
    cost_from_start[start], estimated_total_cost[start] = 0, heuristic(start, end)
    
    while open_set:
        open_set.sort(key=lambda node: estimated_total_cost[node])
        current = open_set.pop(0)
        
        if current == end:
            return reconstruct_path(came_from, current)
        
        for neighbor, cost in current.neighbors:
            if neighbor not in allowed_nodes:
                continue
            tentative_cost = cost_from_start[current] + cost
            if tentative_cost < cost_from_start[neighbor]:
                came_from[neighbor] = current
                cost_from_start[neighbor] = tentative_cost
                estimated_total_cost[neighbor] = cost_from_start[neighbor] + heuristic(neighbor, end)
                if neighbor not in open_set:
                    open_set.append(neighbor)
    
    return None

nodes_way = a_star(nodes[0], nodes[2], nodes)


#print each node in the way as x y rotation

print(nodes)
for node in nodes_way:
    print(node.print_coords())