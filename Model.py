import random
import math


class Model:

    # instance variables
    def __init__(self):
        self.allNodes = []
        self.customers = []
        self.matrix = []
        self.capacity = -1
        self.vehicles = 0

    def BuildModel(self, filename):
        all_lines = list(open(filename, "r"))
        separator = ','
        # read number of vehicles
        line_counter = 0
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        self.vehicles = int(no_spaces[1])

        # read capacity
        line_counter += 1
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        self.capacity = int(no_spaces[1])

        # read number of customer
        line_counter += 1
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        totalCustomers = int(no_spaces[1])

        # read depot node
        line_counter += 3
        ln = all_lines[line_counter]
        no_spaces = ln.split(sep=separator)
        x = float(no_spaces[1])
        y = float(no_spaces[2])
        depot = Node(0, x, y, 0, 0)
        self.allNodes.append(depot)

        # read customers

        for i in range(0, totalCustomers):
            line_counter += 1
            ln = all_lines[line_counter]
            no_spaces = ln.split(sep=separator)
            idd = int(no_spaces[0])
            x = float(no_spaces[1])
            y = float(no_spaces[2])
            demand = int(no_spaces[3])
            serv_time = int(no_spaces[4])
            cust = Node(idd, x, y, demand, serv_time)
            self.allNodes.append(cust)
            self.customers.append(cust)

        # create dist matrix
        rows = len(self.allNodes)
        self.matrix = [[0.0 for x in range(rows)] for y in range(rows)]

        for i in range(0, len(self.allNodes)):
            for j in range(0, len(self.allNodes)):
                a = self.allNodes[i]
                b = self.allNodes[j]
                dist = math.sqrt(math.pow(a.x - b.x, 2) + math.pow(a.y - b.y, 2)) + a.serv_time
                self.matrix[i][j] = dist


class Node:
    def __init__(self, idd, xx, yy, dem, st):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.demand = dem
        self.serv_time = st
        self.isRouted = False
        self.isTabuTillIterator = -1


class Route:
    def __init__(self, dp, cap, m):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.cost = 0
        self.capacity = cap
        self.load = 0
        self.model = m

    def calculateCost(self):
        rt_load = 0
        rt_cumulative_cost = 0
        tot_time = 0
        for i in range(len(self.sequenceOfNodes) - 1):
            from_node = self.sequenceOfNodes[i]
            to_node = self.sequenceOfNodes[i + 1]
            tot_time += self.model.matrix[from_node.ID][to_node.ID]
            rt_cumulative_cost += tot_time
            rt_load += from_node.demand

        return rt_cumulative_cost, rt_load



    def updateCost(self):
        rt_cumulative_cost, rt_load = self.calculateCost()
        self.cost = rt_cumulative_cost
        self.load = rt_load

    def clone(self):
        cloned = Route(None, self.capacity, self.model)
        cloned.cost = self.cost
        cloned.load = self.load
        cloned.sequenceOfNodes = self.sequenceOfNodes.copy()
        return cloned

    def __str__(self):
        s = ''
        for i in range(0, len(self.sequenceOfNodes)-1):
            s += ("%d," % self.sequenceOfNodes[i].ID)
        s += ("%d\r" % self.sequenceOfNodes[-1].ID)
        return s
