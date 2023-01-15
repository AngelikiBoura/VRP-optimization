#from SolutionDrawer import SolDrawer
from Model import *


class Solution:
    def __init__(self, m):
        self.cost = 0.0
        self.routes = []
        self.model = m

    def updateCost(self, forced=False):
        self.cost = 0.0
        if forced:
            for route in self.routes:
                route.updateCost()
        for route in self.routes:
            self.cost += route.cost

    def clone(self):
        cloned = Solution(self.model)
        for route in self.routes:
            clonedRoute = route.clone()
            cloned.routes.append(clonedRoute)
        cloned.cost = self.cost
        return cloned

    def __str__(self):
        s = "Cost:\n"
        s += str(self.cost) + '\n'
        s += "Routes:\n"
        s += str(len(self.routes)) + '\n'
        for rt in self.routes:
            s += str(rt) + '\n'
        return s

    def save(self, filename="Solution.txt"):
        f = open(filename, "w+")
        f.write(str(self))
        f.close()


class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = 10 ** 9


class SwapMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = None

    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = 10 ** 9


class Solver:
    def __init__(self, m):
        self.model = m
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.matrix
        self.capacity = m.capacity
        self.routes = []
        self.vehicles = m.vehicles
        self.sol = None
        self.bestSolution = None
        self.minTabuTenure = 50
        self.maxTabuTenure = 60
        self.tabuTenure = 20

    def solve(self):
        self.InitializeRoutes()
        self.SetRoutedFlagToFalseForAllCustomers()
        self.ApplyNearestNeighborMethod()
        print(self.sol)
        self.TabuSearch(0)
        # self.LocalSearch(1)
        print(self.sol)
        # # self.VND()
        # # print(self.sol)
        return self.sol

    def InitializeRoutes(self):
        self.sol = Solution(self.model)
        for i in range(self.model.vehicles):
            rt = Route(self.depot, self.model.capacity, self.model)
            self.sol.routes.append(rt)

    def SetRoutedFlagToFalseForAllCustomers(self):
        for i in range(0, len(self.model.customers)):
            self.model.customers[i].isRouted = False

    # I find the closest neighbor for a Node with respect to the Node's route capacity
    def FindNearestNeighbor(self, Node, load):
        best_cost = float("inf")
        next_customer = None

        for n in range(0, len(self.model.customers)):
            neighbor = self.model.customers[n]
            if Node.ID == neighbor.ID or neighbor.isRouted or load + neighbor.demand > self.model.capacity:
                continue
            cost_ij = self.model.matrix[Node.ID][neighbor.ID]
            if cost_ij < best_cost:
                next_customer = neighbor
                best_cost = cost_ij

        return next_customer, best_cost

    def ApplyNearestNeighborMethod(self):
        isBooked = [False] * self.model.vehicles
        while not all(isBooked):
            for route in range(len(self.sol.routes)):
                rt: Route = self.sol.routes[route]
                if isBooked[route]:
                    continue
                x, insertion_cost = self.FindNearestNeighbor(rt.sequenceOfNodes[-1], rt.load)
                if x == None:
                    isBooked[route] = True
                    continue
                x.isRouted = True
                rt.sequenceOfNodes.append(x)
                rt.load += x.demand
        self.sol.updateCost(True)



    def FindBestRelocationMove(self,rm,iterator):
        rm = None
        # For each route
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[originRouteIndex]
            # For each node in the route
            for originNodeIndex in range(1, len(rt1.sequenceOfNodes)):
                # For each other route
                for targetRouteIndex in range(0, len(self.sol.routes)):
                    rt2: Route = self.sol.routes[targetRouteIndex]
                    # For each node on the other route
                    for targetNodeIndex in range(0, len(rt2.sequenceOfNodes)):
                        if originRouteIndex == targetRouteIndex and (
                                targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                            continue
                        originRtCostChange = 0
                        targetRtCostChange = 0
                        OriginRtNodesAfterB = len(rt1.sequenceOfNodes) - originNodeIndex - 1
                        TargetRtNodesAfterB = len(rt2.sequenceOfNodes) - targetNodeIndex - 1

                        # The node to move
                        B = rt1.sequenceOfNodes[originNodeIndex]
                        # If the load of the node to move do not fit on the track, skip
                        if originRouteIndex != targetRouteIndex and rt2.load + B.demand > rt2.capacity:
                            continue

                        # If origin node is not the last one
                        if (OriginRtNodesAfterB > 0):
                            # Node before b
                            A1 = rt1.sequenceOfNodes[originNodeIndex - 1]
                            # Node after b
                            C1 = rt1.sequenceOfNodes[originNodeIndex + 1]

                            # Calculate change for origin route if we remove the node
                            originRtCostChange = +(OriginRtNodesAfterB * self.model.matrix[A1.ID][C1.ID]) \
                                                 - ((OriginRtNodesAfterB + 1) * self.model.matrix[A1.ID][B.ID]) \
                                                 - (OriginRtNodesAfterB * self.model.matrix[B.ID][C1.ID])

                            # we remove from the origin route the edge distances between every couple of  the previous nodes until B
                            # because this was carried until node B.
                            for i in range(originNodeIndex - 1):
                                n1 = rt1.sequenceOfNodes[i]
                                n2 = rt1.sequenceOfNodes[i + 1]
                                originRtCostChange -= self.model.matrix[n1.ID][n2.ID]
                        else:
                            A1 = rt1.sequenceOfNodes[originNodeIndex - 1]

                            # Calculate change for origin route if we remove the node
                            originRtCostChange = -self.model.matrix[A1.ID][B.ID]
                            for i in range(originNodeIndex - 1):
                                n1 = rt1.sequenceOfNodes[i]
                                n2 = rt1.sequenceOfNodes[i + 1]
                                originRtCostChange -= self.model.matrix[n1.ID][n2.ID]

                        if (TargetRtNodesAfterB > 0):
                            # node after which we want to put B
                            A2 = rt2.sequenceOfNodes[targetNodeIndex]
                            # node before which we want to put B
                            C2 = rt2.sequenceOfNodes[targetNodeIndex + 1]

                            # Calculate change for target route if we remove the node
                            targetRtCostChange = +((TargetRtNodesAfterB + 1) * self.model.matrix[A2.ID][B.ID]) \
                                                 + (TargetRtNodesAfterB * self.model.matrix[B.ID][C2.ID]) \
                                                 - (TargetRtNodesAfterB * self.model.matrix[A2.ID][C2.ID])

                            # Add the cost that route B should be carrying from the previous couples of nodes.
                            for i in range(targetNodeIndex):
                                n1 = rt2.sequenceOfNodes[i]
                                n2 = rt2.sequenceOfNodes[i + 1]
                                targetRtCostChange += self.model.matrix[n1.ID][n2.ID]
                        else:
                            A2 = rt2.sequenceOfNodes[targetNodeIndex]

                            # Calculate change for target route if we remove the node
                            targetRtCostChange = +self.model.matrix[A2.ID][B.ID]
                            for i in range(targetNodeIndex):
                                n1 = rt2.sequenceOfNodes[i]
                                n2 = rt2.sequenceOfNodes[i + 1]
                                targetRtCostChange += self.model.matrix[n1.ID][n2.ID]

                        moveCost = originRtCostChange + targetRtCostChange

                        if (self.MoveIsTabu(B, iterator, moveCost)):
                            continue
                        if (moveCost < 0 and (not rm or (moveCost < rm.moveCost))):
                            rm = self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex,
                                                              targetNodeIndex, moveCost, originRtCostChange,
                                                              targetRtCostChange, rm)
        return rm

    def StoreBestRelocationMove(self, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost,
                                originRtCostChange, targetRtCostChange, rm: RelocationMove):
        if not rm:
            rm = RelocationMove()
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.costChangeOriginRt = originRtCostChange
        rm.costChangeTargetRt = targetRtCostChange
        rm.moveCost = moveCost
        return rm

    def AssessRelocationMove(self, rm: RelocationMove):
        originRt = self.sol.routes[rm.originRoutePosition].clone()
        if rm.originRoutePosition == rm.targetRoutePosition:
            targetRt = originRt
        else:
            targetRt = self.sol.routes[rm.targetRoutePosition].clone()

        initial_cost = originRt.cost + targetRt.cost

        B = originRt.sequenceOfNodes[rm.originNodePosition]
        del originRt.sequenceOfNodes[rm.originNodePosition]

        if originRt == targetRt:
            if rm.targetNodePosition == len(targetRt.sequenceOfNodes) - 1:
                targetRt.sequenceOfNodes.append(B)
            else:
                if (rm.originNodePosition < rm.targetNodePosition):
                    targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
                else:
                    targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)

        else:
            if rm.targetNodePosition == len(targetRt.sequenceOfNodes) - 1:
                targetRt.sequenceOfNodes.append(B)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)

        originRt.updateCost()
        targetRt.updateCost()
        rm.moveCost = 0 - (initial_cost - (originRt.cost + targetRt.cost))
        return rm.moveCost

    def ApplyRelocationMove(self, rm: RelocationMove, iterator):
        originRt = self.sol.routes[rm.originRoutePosition]
        targetRt = self.sol.routes[rm.targetRoutePosition]
        # remove origin and target routes' cost from solution
        self.sol.cost -= originRt.cost
        self.sol.cost -= targetRt.cost

        B = originRt.sequenceOfNodes[rm.originNodePosition]
        # delete node B from initial route
        del originRt.sequenceOfNodes[rm.originNodePosition]

        if originRt == targetRt:
            if rm.targetNodePosition == len(targetRt.sequenceOfNodes) - 1:
                targetRt.sequenceOfNodes.append(B)
            else:
                if (rm.originNodePosition < rm.targetNodePosition):
                    targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
                else:
                    targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            # i add the change of the cost in the origin route's cost
            originRt.cost += rm.moveCost

        else:
            if rm.targetNodePosition == len(targetRt.sequenceOfNodes) - 1:
                targetRt.sequenceOfNodes.append(B)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            # I add the change of cost in target and original route's costs
            originRt.cost += rm.costChangeOriginRt
            targetRt.cost += rm.costChangeTargetRt
            # I change the demand in both routes
            originRt.load -= B.demand
            targetRt.load += B.demand
        # I add again the costs of the changed routes in the solution.
        self.sol.cost += originRt.cost
        self.sol.cost += targetRt.cost
        self.SetTabuIterator(B, iterator)


    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost,
                          costChangeFirstRoute, costChangeSecondRoute, sm):

        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.costChangeFirstRt = costChangeFirstRoute
        sm.costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    def ApplySwapMove(self, sm, iterator):
        rt1 = self.sol.routes[sm.positionOfFirstRoute]
        rt2 = self.sol.routes[sm.positionOfSecondRoute]
        b1 = rt1.sequenceOfNodes[sm.positionOfFirstNode]
        b2 = rt2.sequenceOfNodes[sm.positionOfSecondNode]
        #  subtract the two routes' cost from the solution
        self.sol.cost -= rt1.cost
        self.sol.cost -= rt2.cost
        # swap the two nodes
        rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
        rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1

        if (rt1 == rt2):
            # calculate the cost and load for the route that changed
            rt1.cost, rt1.load = rt1.calculateCost()

        else:
            # calculate the cost and load for the routes that changed
            rt1.cost, rt1.load = rt1.calculateCost()
            rt2.cost, rt2.load = rt2.calculateCost()

        # put the cost again in the solution
        self.sol.cost += rt1.cost
        self.sol.cost += rt2.cost
        self.sol.updateCost(True)
        print(self.sol.cost)
        self.SetTabuIterator(b1, iterator)
        self.SetTabuIterator(b2, iterator)


    def InitializeOperators(self, rm, sm):
        rm.Initialize()
        sm.Initialize()
        # top.Initialize()

    def FindBestSwapMove(self,sm, iterator):
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1: Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range(firstRouteIndex, len(self.sol.routes)):
                rt2: Route = self.sol.routes[secondRouteIndex]
                for firstNodeIndex in range(1, len(rt1.sequenceOfNodes)):
                    # for every other route start from the first node
                    startOfSecondNodeIndex = 1
                    # for the same route start from the next node of the first node
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    for secondNodeIndex in range(startOfSecondNodeIndex, len(rt2.sequenceOfNodes)):

                        a1 = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        # c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        # c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                        firstRtNodesAfterB1 = len(rt1.sequenceOfNodes) - firstNodeIndex - 1
                        secondRtNodesAfterB2 = len(rt2.sequenceOfNodes) - secondNodeIndex - 1
                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None

                        if  firstRtNodesAfterB1 > 0 and secondRtNodesAfterB2 > 0:
                            c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]
                            c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                        # if first=second route
                            if rt1 == rt2:
                                # if first node comes exactly before the second node
                                if firstNodeIndex == secondNodeIndex - 1:
                                    # case of consecutive nodes swap
                                    costRemoved = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                                  (secondRtNodesAfterB2 + 1) * self.distanceMatrix[b1.ID][b2.ID] + \
                                                  secondRtNodesAfterB2 * self.distanceMatrix[b2.ID][c2.ID]
                                    costAdded = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                                (firstRtNodesAfterB1 + 1) * self.distanceMatrix[b2.ID][b1.ID] + \
                                                secondRtNodesAfterB2 * self.distanceMatrix[b1.ID][c2.ID]
                                    moveCost = costAdded - costRemoved
                                else:

                                    costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                                   firstRtNodesAfterB1*self.distanceMatrix[b1.ID][c1.ID]
                                    costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                                 firstRtNodesAfterB1 * self.distanceMatrix[b2.ID][c1.ID]
                                    costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID] + \
                                                   secondRtNodesAfterB2 * self.distanceMatrix[b2.ID][c2.ID]
                                    costAdded2 = (secondRtNodesAfterB2 + 1)*self.distanceMatrix[a2.ID][b1.ID] +\
                                                 secondRtNodesAfterB2*self.distanceMatrix[b1.ID][c2.ID]
                                    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                            else:
                                if rt1.load - b1.demand + b2.demand > self.capacity:
                                    continue
                                if rt2.load - b2.demand + b1.demand > self.capacity:
                                    continue

                                costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                               firstRtNodesAfterB1 * self.distanceMatrix[b1.ID][c1.ID]
                                costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                             firstRtNodesAfterB1 * self.distanceMatrix[b2.ID][c1.ID]
                                costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID] + \
                                               secondRtNodesAfterB2 * self.distanceMatrix[b2.ID][c2.ID]
                                costAdded2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b1.ID] + \
                                             secondRtNodesAfterB2 * self.distanceMatrix[b1.ID][c2.ID]

                                costChangeFirstRoute = costAdded1 - costRemoved1
                                costChangeSecondRoute = costAdded2 - costRemoved2

                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        elif firstRtNodesAfterB1 > 0 and secondRtNodesAfterB2 == 0:
                            c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]
                            if rt1 == rt2:

                                # if first node comes exactly before the second node
                                if firstNodeIndex == secondNodeIndex - 1:
                                    # case of consecutive nodes swap
                                    costRemoved = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                                  (secondRtNodesAfterB2 + 1) * self.distanceMatrix[b1.ID][b2.ID]

                                    costAdded = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                                (firstRtNodesAfterB1 + 1) * self.distanceMatrix[b2.ID][b1.ID]

                                    moveCost = costAdded - costRemoved
                                else:

                                    costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                                   firstRtNodesAfterB1*self.distanceMatrix[b1.ID][c1.ID]
                                    costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                                 firstRtNodesAfterB1 * self.distanceMatrix[b2.ID][c1.ID]
                                    costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID]

                                    costAdded2 = (secondRtNodesAfterB2 + 1)*self.distanceMatrix[a2.ID][b1.ID]
                                    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                            else:
                                if rt1.load - b1.demand + b2.demand > self.capacity:
                                    continue
                                if rt2.load - b2.demand + b1.demand > self.capacity:
                                    continue

                                costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                               firstRtNodesAfterB1 * self.distanceMatrix[b1.ID][c1.ID]
                                costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                             firstRtNodesAfterB1 * self.distanceMatrix[b2.ID][c1.ID]
                                costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID]
                                costAdded2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b1.ID]

                                costChangeFirstRoute = costAdded1 - costRemoved1
                                costChangeSecondRoute = costAdded2 - costRemoved2

                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        elif firstRtNodesAfterB1 == 0 and secondRtNodesAfterB2 > 0:
                            c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                            # if first=second route
                            if rt1 == rt2:

                                # if first node comes exactly before the second node
                                if firstNodeIndex == secondNodeIndex - 1:
                                    # case of consecutive nodes swap
                                    costRemoved = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                                  (secondRtNodesAfterB2 + 1) * self.distanceMatrix[b1.ID][b2.ID] + \
                                                  secondRtNodesAfterB2 * self.distanceMatrix[b2.ID][c2.ID]
                                    costAdded = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                                (firstRtNodesAfterB1 + 1) * self.distanceMatrix[b2.ID][b1.ID] + \
                                                secondRtNodesAfterB2 * self.distanceMatrix[b1.ID][c2.ID]

                                    moveCost = costAdded - costRemoved
                                else:

                                    costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID]
                                    costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID]
                                    costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID] + \
                                                   secondRtNodesAfterB2 * self.distanceMatrix[b2.ID][c2.ID]
                                    costAdded2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b1.ID] + \
                                                 secondRtNodesAfterB2 * self.distanceMatrix[b1.ID][c2.ID]
                                    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                            else:
                                if rt1.load - b1.demand + b2.demand > self.capacity:
                                    continue
                                if rt2.load - b2.demand + b1.demand > self.capacity:
                                    continue

                                costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID]
                                costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID]
                                costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID] + \
                                               secondRtNodesAfterB2 * self.distanceMatrix[b2.ID][c2.ID]
                                costAdded2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b1.ID] + \
                                             secondRtNodesAfterB2 * self.distanceMatrix[b1.ID][c2.ID]

                                costChangeFirstRoute = costAdded1 - costRemoved1
                                costChangeSecondRoute = costAdded2 - costRemoved2
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                        else:
                            # if first=second route
                            if rt1 == rt2:

                                # if first node comes exactly before the second node
                                if firstNodeIndex == secondNodeIndex - 1:
                                    # case of consecutive nodes swap
                                    costRemoved = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID] + \
                                                  (secondRtNodesAfterB2 + 1) * self.distanceMatrix[b1.ID][b2.ID]
                                    costAdded = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID] + \
                                                (firstRtNodesAfterB1 + 1) * self.distanceMatrix[b2.ID][b1.ID]

                                    moveCost = costAdded - costRemoved
                                else:

                                    costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID]
                                    costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID]
                                    costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID]
                                    costAdded2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b1.ID]

                                    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                            else:
                                if rt1.load - b1.demand + b2.demand > self.capacity:
                                    continue
                                if rt2.load - b2.demand + b1.demand > self.capacity:
                                    continue

                                costRemoved1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b1.ID]
                                costAdded1 = (firstRtNodesAfterB1 + 1) * self.distanceMatrix[a1.ID][b2.ID]
                                costRemoved2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b2.ID]
                                costAdded2 = (secondRtNodesAfterB2 + 1) * self.distanceMatrix[a2.ID][b1.ID]

                                costChangeFirstRoute = costAdded1 - costRemoved1
                                costChangeSecondRoute = costAdded2 - costRemoved2
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        if self.MoveIsTabu(b1, iterator, moveCost) or self.MoveIsTabu(b2, iterator, moveCost):
                            continue
                        if moveCost < sm.moveCost:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def TabuSearch(self, operator):
        solution_cost_trajectory = []
        random.seed(2)
        self.bestSolution = self.sol.clone()
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        # top:TwoOptMove = TwoOptMove()

        #SolDrawer.draw(0, self.sol, self.allNodes)

        while terminationCondition is False:
            operator = random.randint(0,2)

            rm.Initialize()
            sm.Initialize()
            # top.Initialize()

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm, localSearchIterator)
                if rm.originRoutePosition is not None:
                    self.ApplyRelocationMove(rm, localSearchIterator)
            # Swaps
            elif operator == 1:
                self.FindBestSwapMove(sm, localSearchIterator)
                if sm.positionOfFirstRoute is not None:
                    self.ApplySwapMove(sm, localSearchIterator)
            # elif operator == 2:
            #     self.FindBestTwoOptMove(top, localSearchIterator)
            #     if top.positionOfFirstRoute is not None:
            #         self.ApplyTwoOptMove(top, localSearchIterator)

            # self.ReportSolution(self.sol)
            # self.TestSolution()
            # solution_cost_trajectory.append(self.sol.cost)

            print(localSearchIterator, self.sol.cost, self.bestSolution.cost)

            if (self.sol.cost < self.bestSolution.cost):
                self.bestSolution = self.sol.clone()

            # SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            localSearchIterator = localSearchIterator + 1

            if localSearchIterator > 10000:
                terminationCondition = True

        # SolDrawer.draw('final_ts', self.bestSolution, self.allNodes)
        # SolDrawer.drawTrajectory(solution_cost_trajectory)

        self.sol = self.bestSolution

    def MoveIsTabu(self, n: Node, iterator, moveCost):
        if moveCost + self.sol.cost < self.bestSolution.cost - 0.001:
            return False
        if iterator < n.isTabuTillIterator:
            return True
        return False

    def SetTabuIterator(self, n: Node, iterator):
        # n.isTabuTillIterator = iterator + self.tabuTenure
        n.isTabuTillIterator = iterator + random.randint(self.minTabuTenure, self.maxTabuTenure)
