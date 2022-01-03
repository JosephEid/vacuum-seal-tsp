import os
import numpy as np

from math import sqrt
from statistics import mean
from matplotlib import pyplot


class Circle(object):
    """
    Circle class representing a circle shrinking around nodes
    """

    def __init__(self, centre_x, centre_y, radius):
        self.centre_x = centre_x
        self.centre_y = centre_y
        self.radius = radius
        self.plot = pyplot.Circle((self.centre_x, self.centre_y), self.radius, fill=False)


class Node(object):
    """
    Node class representing cities within the instance
    """

    def __init__(self, x, y, init_pos):
        self.x = x
        self.y = y
        self.hit = False
        self.init_pos = init_pos
        self.final_pos = None
        self.distance_from_centre = None

    def distance(self, node):
        return sqrt((self.x - node.x) ** 2 + (self.y - node.y) ** 2)


class Instance(object):
    """
    Instance class representing the TSP Instance
    """

    def __init__(self, nodes):
        self.furthest = None
        self.centroid = None
        self.nodes = nodes

    def run_vacuum_seal(self):
        self.centroid = self.get_centre_point()
        self.furthest = self.get_farthest_distance()
        self.plot_graph()

    def get_centre_point(self):
        x_mean = mean([x.x for x in self.nodes])
        y_mean = mean([x.y for x in self.nodes])
        return Node(x_mean, y_mean, None)

    def plot_graph(self):
        fig = pyplot.figure()
        axis = fig.add_subplot(1, 1, 1)
        axis.text(self.centroid.x, self.centroid.y, "c")
        # by_dist = sorted(self.nodes, key=lambda x: x.distance_from_centre, reverse=True)

        convex_hull = convex(self.nodes)
        print(convex_hull)
        # for points in convex_hull:
        #     print(points)

        for node1 in self.nodes:
            axis.scatter(node1.x, node1.y)
            axis.text(node1.x, node1.y + 5, f"{node1.init_pos}")

        for i in range(len(convex_hull)-1):
            axis.plot([convex_hull[i].x,convex_hull[i+1].x], [convex_hull[i].y,convex_hull[i+1].y])
            if i == len(convex_hull)-2:
                axis.plot([convex_hull[i+1].x, convex_hull[0].x], [convex_hull[i+1].y, convex_hull[0].y])
        #     for node2 in self.nodes:
        #         if node1.init_pos != node2.init_pos:
        #             axis.plot([node1.x,node2.x], [node1.y, node2.y])

        # circle = Circle(self.centroid.x, self.centroid.y, self.furthest)
        # pyplot.gca().add_patch(circle.plot)
        #
        # while circle.plot.radius > 0:
        #     circle.plot.radius -= 1
        #     pyplot.pause(0.0005)

        pyplot.show()

    def get_farthest_distance(self):
        for node in nodes:
            node.distance_from_centre = node.distance(self.centroid)

        return np.max([x.distance_from_centre for x in self.nodes])


def direction(a, b, c):
    orientation = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)

    if orientation == 0:
        return 0
    elif orientation > 0:
        return 1
    else:
        return 2


def convex(points):
    if len(points) < 3:
        return

    convex_hull = []
    current_point = min(points, key=lambda point: point.x)
    hull = []
    a = points.index(current_point)

    while True:
        hull.append(a)
        b = (a + 1) % len(points)

        for i in range(len(points)):
            if direction(points[a], points[i], points[b]) == 2:
                b = i
        a = b
        if a == current_point.init_pos:
            break
    for n in hull:
        convex_hull.append(points[n])

    return convex_hull


def create_nodes(instance_name):
    """
    Creates a list of nodes for a standard non-custom TSP Instance
    """
    # Find the tsp instance file and extract its extension
    dir = os.path.dirname(os.path.realpath(__file__))
    file = open(dir + '/instances/' + instance_name)
    ext = name.split('.')[1]
    nodes = []  # Array storing nodes

    # If .csv then just read nodes line by line
    if ext == "csv":
        for i, line in enumerate(file):  # enumerate used to obtain line numbers and thus node numbers
            coords = line.rsplit()[0].split(",")
            x = int(coords[0])
            y = int(coords[1])
            nodes.append(Node(x, y, i))
    elif ext == "tsp":
        # If .tsp then the format of the file changes and needs to be read differently.
        file.readline()
        file.readline()
        file.readline()
        no_nodes = int(file.readline().strip().split()[1])
        file.readline()
        file.readline()
        file.readline()

        for i in range(0, no_nodes):
            coords = file.readline().strip().split()[1:]
            x = float(coords[0])
            y = float(coords[1])

            nodes.append(Node(x, y, i))

    return nodes


if __name__ == "__main__":
    name = "bayg29.tsp"
    nodes = create_nodes(name)
    i = Instance(nodes)
    i.run_vacuum_seal()
