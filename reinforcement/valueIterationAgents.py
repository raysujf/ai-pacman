# valueIterationAgents.py
# -----------------------
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


# valueIterationAgents.py
# -----------------------
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


import mdp, util

from learningAgents import ValueEstimationAgent
import collections

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.runValueIteration()

    def runValueIteration(self):
        # Write value iteration code here
        "*** YOUR CODE HERE ***"
        """
        for action in self.mdp.getPossibleActions(self.mdp.getStates()[2]):
            print(action)
            print(self.mdp.getTransitionStatesAndProbs(self.mdp.getStates()[2],action))
        """
        """
        for state in self.mdp.getStates():
            result=[]
            for action in self.mdp.getPossibleActions(state):
                result.append(self.getQValue(state,action))
            print(result)
            
"""
        temp={}
        while self.iterations:
            for state in self.mdp.getStates():
                if self.mdp.isTerminal(state):continue
                temp[state]=max([self.getQValue(state,action) for action in self.mdp.getPossibleActions(state)])
            self.iterations-=1
            for i in temp:
                self.values[i]=temp[i]

        return self.values
        

        """
        for i in self.mdp.getStates():
            for action in self.mdp.getPossibleActions(i):
                print(action,i)
                print(self.mdp.getTransitionStatesAndProbs(i, action))
                print(self.mdp.isTerminal(i))
"""

    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** YOUR CODE HERE ***"
        
        arr=self.mdp.getTransitionStatesAndProbs(state,action)
        result=0
        for i in arr:
            next_state,prob=i[0],i[1]
            reward=self.mdp.getReward(state,action,next_state)
            result+=i[1]*(reward+self.discount*self.getValue(next_state))
            """
            print("#",str(state),str(action))
            print(next_state,prob,reward,self.getValue(next_state))
            """
        return result
        
        util.raiseNotDefined()

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        """

        for action in self.mdp.getPossibleActions(state):
            
        util.raiseNotDefined()
        """
        result=[]
        if self.mdp.isTerminal(state):return None
        for action in self.mdp.getPossibleActions(state):
            result.append((self.computeQValueFromValues(state,action),action))
        return max(result)[1]

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)

class AsynchronousValueIterationAgent(ValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        An AsynchronousValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs cyclic value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 1000):
        """
          Your cyclic value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy. Each iteration
          updates the value of only one state, which cycles through
          the states list. If the chosen state is terminal, nothing
          happens in that iteration.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state)
              mdp.isTerminal(state)
        """
        self.now=-1
        ValueIterationAgent.__init__(self, mdp, discount, iterations)


    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        temp={}
        while self.iterations:
            self.now=(self.now+1)%len(self.mdp.getStates())
            state=self.mdp.getStates()[self.now]
            if self.mdp.isTerminal(state):
                self.iterations-=1
                continue
            self.values[state]=max([self.getQValue(state,action) for action in self.mdp.getPossibleActions(state)])
            self.iterations-=1
            
        return self.values


class PrioritizedSweepingValueIterationAgent(AsynchronousValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A PrioritizedSweepingValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs prioritized sweeping value iteration
        for a given number of iterations using the supplied parameters.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100, theta = 1e-5):
        """
          Your prioritized sweeping value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy.
        """
        self.theta = theta
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        while self.iterations:
            differences=util.PriorityQueue()
            predeccssors={}
            for state in self.mdp.getStates():
                if self.mdp.isTerminal(state):continue
                predeccsors=[self.getQValue(state,action) for action in self.mdp.getPossibleActions(state)]
                MAX=max([i-self.values[state] for i in predeccsors])
                diff=(self.values[state]+MAX,state)
                differences.push(diff,-abs(MAX))
            find=differences.pop()
            self.values[find[1]]=find[0]
            self.iterations-=1
