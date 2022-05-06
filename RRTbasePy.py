"""
This code is the class implementation of RRT algorithm
"""

import random
import math
import pygame


class RRTMap:
    """
    Class for drawing the map, and path that is calculated
    """

    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        """
        :param start: start coordinates (Tuple)
        :param goal: goal coordinates (Tuple)
        :param MapDimensions: Dimension of the map (width and height)
        :param obsdim: obstacles dimension
        :param obsnum: Number of obstacles
        """
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.maph, self.mapw = self.MapDimensions

        # window setting
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.fill((255, 255, 255))
        self.NodeRad = 2  # radius of the nodes
        self.nodeThickness = 0  # thickness of the nodes
        self.edgeThickness = 1  # thickenss of the edges

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # Color definitions
        self.Grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawMap(self, obstacles):
        """
        Draws the start and goal as circle and draws the map
        :param obstacles: obstacles list (list of pygame rect objects)
        """
        pygame.draw.circle(self.map, self.Green, self.start, self.NodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Red, self.goal, self.NodeRad + 20, 1)
        self.drawObs(obstacles)

    def drawPath(self):
        pass

    def drawObs(self, obstacles):
        """
        Draws the given obstacles list
        :param obstacles: obs. list (list of pygame rect)
        """
        obstacleList = obstacles.copy()
        while len(obstacleList) > 0:
            obstacle = obstacleList.pop(0)
            pygame.draw.rect(self.map, self.Grey, obstacle)


class RRTGraph:
    """
    Main RRT algorithm class
    """

    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False # flag to see if the RRT tree has reached the goal
        self.maph, self.mapw = MapDimensions
        # tree data structures
        self.x = []
        self.y = []
        self.parent = []
        # initalize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        # the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        # path
        self.goalstate = None  # flag to check whether the tree reached the goal or not
        self.path = []  # list to hold the calculated path

    def makeRandomRect(self):
        """
        Creates upper corner position for random rectangles (used for obstacle generation)
        :return: upper corner x, y coord.  (Tuple)
        """
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))
        return (uppercornerx, uppercornery)

    def makeobs(self):
        """
        Create obstacle and store in a list
        :return: obstacle list (pygame rect objects)
        """
        obs = []

        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        print(obs)
        return obs

    def add_node(self, n, x, y):
        """
        adds node to the rrt graph
        :param n: id number of the node
        :param x: x-coord
        :param y: y-coord
        """
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        """
        removes the node from the RRT graph
        :param n: id of the node
        """
        self.x.pop(n)
        self.y.pop(n)

    def number_of_nodes(self):
        """
        :return: number of the nodes in the graph
        """
        return len(self.x)

    def add_edge(self, parent, child):
        """
        addes an edge between a parent and a child
        :param parent: used as element
        :param child: used as index
        """
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        """
        :param n: index of the child
        """
        self.parent.pop(n)

    def number_of_edge(self):
        pass

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self):
        pass

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def corssObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = x1 * u + x2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.corssObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self):
        pass

    def path_to_goal(self):
        pass

    def getPathCoords(self):
        pass

    def bias(self):
        pass

    def expand(self):
        pass
