import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.maph, self.mapw = self.MapDimensions

        # window setting
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw,self.maph))
        self.map.fill((255,255,255))
        self.NodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles=[]
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #Colors
        self.Grey = (70,70,70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255,255,255)

    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.Green, self.start, self.NodeRad+5,  0)
        pygame.draw.circle(self.map, self.Red, self.goal, self.NodeRad+20, 1)
        self.drawObs(obstacles)

    def drawPath(self):
        pass
    def drawObs(self, obstacles):
        obstacleList = obstacles.copy()
        while len(obstacleList) > 0:
            obstacle = obstacleList.pop(0)
            pygame.draw.rect(self.map, self.Grey, obstacle)



class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
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
        self.goalstate = None
        self.path = []

    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw-self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))
        return (uppercornerx, uppercornery)

    def makeobs(self):
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
        return obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_edge(self):
        pass

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        return (px+py)**(0.5)

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
            if rectang.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True

    def corssObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i/100
                x = x1 * u + x2 * (1-u)
                y = x1 * u + x2 * (1-u)
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
