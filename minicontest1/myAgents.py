# myAgents.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

from game import Agent
from searchProblems import PositionSearchProblem

import util
import time
import search

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

"""
class OptimalProblem(PositionSearchProblem):
    
    def __init__(self, gameState, agentIndex):
        self.food = gameState.getFood()
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPositions()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def getSuccessors(self,state):
        successors=[]
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y=state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

    def isGoalState(self):
        for i in self.food:
            for j in i:
                if i:return False
        return True

def findfood(state,food,wall):
    result=util.Queue()
    check=set()
    result.push((state,[]))
    while not result.isEmpty():
        element=result.pop()
        if food[element[0][0]][element[0][1]]:
            return element[1]
        if element[0] not in check:
            check.add(element[0])
            for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
                x,y = state
                dx, dy = Actions.directionToVector(action)
                nextx, nexty = int(x + dx), int(y + dy)
                if not wall[nextx][nexty]:
                    nextState=(nextx,nexty)
                    result.push((nextState,element[1]+[action]))

def bfs(problem):
    result=util.Queue()
    check=set()
    result.push((problem.startState,[]))
    while not result.isEmpty():
        element=result.pop()
        if problem.isGoalState():
            return element[1]
        if element[0] not in check:
            check.add(element[0])
            for i in problem.getSuccessors(element[0]):
                result.push((i[0],element[1]+[i[1]]))
"""

def BFS(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    result=util.Queue()
    num=problem.getStartState()
    check=set()
    result.push((num,[]))
    while not result.isEmpty():
        element=result.pop()
        if problem.isGoalState(element[0]):
            return element
        if element[0] not in check:
            check.add(element[0])
            for i in problem.getSuccessors(element[0]):
                result.push((i[0],element[1]+[i[1]]))

class foodproblem(PositionSearchProblem):

    def __init__(self, gameState, agentIndex, food, startstate):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = food
        self.walls = gameState.getWalls()
        self.startState = startstate
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        x,y = state
        return self.food[x][y] or self.food.count()==0

FOOD=False
                
class MyAgent(Agent):
    """
    Implementation of your agent.
    """

    def findPathToClosestDot(self, gameState, food, startstate):
        problem = foodproblem(gameState, self.index, food, startstate)
        return BFS(problem)

    
    def getAction(self, state):
        """
        Returns the next action the agent will take
        """
        global FOOD
        if not FOOD:
            FOOD=state.getFood()
        if self.path:return self.path.pop(0)
        startstate=state.getPacmanPosition(self.index)
        for i in range(6):
            result=self.findPathToClosestDot(state,FOOD,startstate)
            x,y=result[0]
            startstate=result[0]
            FOOD[x][y]=False
            self.path=self.path+result[1]
        if not self.path:
            from game import Directions
            return Directions.STOP
        return self.path.pop(0)

        "*** YOUR CODE HERE ***"
        

        raise NotImplementedError()

    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """

        "*** YOUR CODE HERE"
        self.path=[]

"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class ClosestDotAgent(Agent):

    def findPathToClosestDot(self, gameState):
        
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)

        "*** YOUR CODE HERE ***"
        
        return search.breadthFirstSearch(problem)
        util.raiseNotDefined()

    def getAction(self, state):
        return self.findPathToClosestDot(state)[0]


class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        return self.food[x][y] or self.food.count()==0
        util.raiseNotDefined()

"""
def check(arr):
    for i in arr:
        if i.count(True)!=0:return True
    return False
        
class ClosestDotAgent(Agent):
    
    def __init__(self, index=0):
        self.index = index
        self.initialize()
        self.check=True
        
    def helper(self,gameState):
        result=[]
        num=gameState.getNumPacmanAgents()
        find=gameState.getWidth()//num+1
        for i in range(num):
            result.append((i*find,(i+1)*find))
        self.arr=result
        
    def findPathToClosestDot(self, gameState):
        if self.check:
            self.helper(gameState)
            self.check=False
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)
        left,right=self.arr[self.index][0],self.arr[self.index][1]
        if check(food[left:right]):
            return other(problem,left,right)
        return search.breadthFirstSearch(problem)
        util.raiseNotDefined()


    def getAction(self, state):
        if not self.findPathToClosestDot(state):
            print(self.hi)
        return self.findPathToClosestDot(state)[0]

def other(problem,left,right):
    result=util.Queue()
    num=problem.getStartState()
    check=set()
    result.push((num,[]))
    while not result.isEmpty():
        element=result.pop()
        if element[0][0]>=left and element[0][0]<right and problem.isGoalState(element[0]):
            return element[1]
        if element[0] not in check:
            check.add(element[0])
            for i in problem.getSuccessors(element[0]):
                result.push((i[0],element[1]+[i[1]]))
"""
