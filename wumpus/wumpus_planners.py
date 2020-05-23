# wumpus_planners.py
# ------------------
# Licensing Information:
# Please DO NOT DISTRIBUTE OR PUBLISH solutions to this project.
# You are free to use and extend these projects for EDUCATIONAL PURPOSES ONLY.
# The Hunt The Wumpus AI project was developed at University of Arizona
# by Clay Morrison (clayton@sista.arizona.edu), spring 2013.
# This project extends the python code provided by Peter Norvig as part of
# the Artificial Intelligence: A Modern Approach (AIMA) book example code;
# see http://aima.cs.berkeley.edu/code.html
# In particular, the following files come directly from the AIMA python
# code: ['agents.py', 'logic.py', 'search.py', 'utils.py']
# ('logic.py' has been modified by Clay Morrison in locations with the
# comment 'CTM')
# The file ['minisat.py'] implements a slim system call wrapper to the minisat
# (see http://minisat.se) SAT solver, and is directly based on the satispy
# python project, see https://github.com/netom/satispy .

from wumpus_environment import *
from wumpus_kb import *
import search

#-------------------------------------------------------------------------------
# Distance fn
#-------------------------------------------------------------------------------

def manhattan_distance_with_heading(current, target):
    """
    Return the Manhattan distance + any turn moves needed
        to put target ahead of current heading
    current: (x,y,h) tuple, so: [0]=x, [1]=y, [2]=h=heading)
    heading: 0:^:north 1:<:west 2:v:south 3:>:east
    """
    md = abs(current[0] - target[0]) + abs(current[1] - target[1])
    if current[2] == 0:   # heading north
        # Since the agent is facing north, "side" here means
        # whether the target is in a row above or below (or
        # the same) as the agent.
        # (Same idea is used if agent is heading south)
        side = (current[1] - target[1])
        if side > 0:
            md += 2           # target is behind: need to turns to turn around
        elif side <= 0 and current[0] != target[0]:
            md += 1           # target is ahead but not directly: just need to turn once
        # note: if target straight ahead (curr.x == tar.x), no turning required
    elif current[2] == 1: # heading west
        # Now the agent is heading west, so "side" means
        # whether the target is in a column to the left or right
        # (or the same) as the agent.
        # (Same idea is used if agent is heading east)
        side = (current[0] - target[0])
        if side < 0:
            md += 2           # target is behind
        elif side >= 0 and current[1] != target[1]:
            md += 1           # target is ahead but not directly
    elif current[2] == 2: # heading south
        side = (current[1] - target[1])
        if side < 0:
            md += 2           # target is behind
        elif side >= 0 and current[0] != target[0]:
            md += 1           # target is ahead but not directly
    elif current[2] == 3: # heading east
        side = (current[0] - target[0])
        if side > 0:
            md += 2           # target is behind
        elif side <= 0 and current[1] != target[1]:
            md += 1           # target is ahead but not directly
    return md


#-------------------------------------------------------------------------------
# Plan Route
#-------------------------------------------------------------------------------

def plan_route(current, heading, goals, allowed):
    """
    Given:
       current location: tuple (x,y)
       heading: integer representing direction
       gaals: list of one or more tuple goal-states
       allowed: list of locations that can be moved to
    ... return a list of actions (no time stamps!) that when executed
    will take the agent from the current location to one of (the closest)
    goal locations
    You will need to:
    (1) Construct a PlanRouteProblem that extends search.Problem
    (2) Pass the PlanRouteProblem as the argument to astar_search
        (search.astar_search(Problem)) to find the action sequence.
        Astar returns a node.  You can call node.solution() to exract
        the list of actions.
    NOTE: represent a state as a triple: (x, y, heading)
          where heading will be an integer, as follows:
          0='north', 1='west', 2='south', 3='east'
    """

    # Ensure heading is a in integer form
    if isinstance(heading,str):
        heading = Explorer.heading_str_to_num[heading]

    if goals and allowed:
        prp = PlanRouteProblem((current[0], current[1], heading), goals, allowed)
        # NOTE: PlanRouteProblem will include a method h() that computes
        #       the heuristic, so no need to provide here to astar_search()
        node = search.astar_search(prp)
        if node:
            return node.solution()
    
    # no route can be found, return empty list
    return []

#-------------------------------------------------------------------------------

class PlanRouteProblem(search.Problem):
    def __init__(self, initial, goals, allowed):
        """ Problem defining planning of route to closest goal
        Goal is generally a location (x,y) tuple, but state will be (x,y,heading) tuple
        initial = initial location, (x,y) tuple
        goals   = list of goal (x,y) tuples
        allowed = list of state (x,y) tuples that agent could move to """
        self.initial = initial # initial state
        self.goals = goals     # list of goals that can be achieved
        self.allowed = allowed # the states we can move into

    def h(self,node):
        """
        Heuristic that will be used by search.astar_search()
        """
        "*** YOUR CODE HERE ***"
        minD = 9999999
        for goal in self.goals:
            minD = min(minD, manhattan_distance_with_heading(node.state, goal))
        return minD

    def actions(self, state):
        """
        Return list of allowed actions that can be made in state
        """
        "*** YOUR CODE HERE ***"
        forward,right,left = state,state,state
        x, y, direction = state[0:3]
        # 0='north', 1='west', 2='south', 3='east'
        if direction is 0:
            forward = (x, y+1, 0)
            left = (x, y, 1)
            right = (x, y, 3)
        elif direction is 1:
            forward = (x-1, y, 1)
            left = (x, y, 2)
            right = (x, y, 0)
        elif direction is 2:
            forward = (x, y-1, 2)
            left = (x, y, 3)
            right = (x, y, 1)
        elif direction is 3:
            forward = (x+1, y, 3)
            left = (x, y, 0)
            right = (x, y, 2)
        minF = 99999
        minL = 99999
        minR = 99999
        for goal in self.goals:
            dfwd = 99999
            allowedD = self.allowed
            if (forward[0], forward[1]) in allowedD:
                dfwd = manhattan_distance_with_heading(forward, goal)
            minF = min(minF, dfwd)
            minR = min(minR, manhattan_distance_with_heading(right, goal))
            minL = min(minL, manhattan_distance_with_heading(left, goal))
        if minF <= minR and minF <= minL:
            return ['Forward']
        elif minR <= minF and minR <= minL:
            return ['TurnRight']
        elif minL <= minF and minL <= minR:
            return ['TurnLeft']
        return None



    def result(self, state, action):
        """
        Return the new state after applying action to state
        """
        "*** YOUR CODE HERE ***"
        x, y, direction = state[0:3]
        # 0='north', 1='west', 2='south', 3='east'
        if proposition_bases_actions.index(action) is 0:
            if direction is 0:
                return x, y+1, 0
            elif direction is 1:
                return x-1, y, 1
            elif direction is 2:
                return x, y - 1, 2
            elif direction is 3:
                return x + 1, y, 3
        elif proposition_bases_actions.index(action) is 4:
            return (x, y, 0) if (direction+1)>3 else (x, y, direction+1)
        elif proposition_bases_actions.index(action) is 5:
            return (x, y, 3) if (direction-1)<0 else (x, y, direction-1)
        return state

    def goal_test(self, state):
        """
        Return True if state is a goal state
        """
        "*** YOUR CODE HERE ***"
        x, y = state[0:2]
        return True if (x, y) in self.goals else False

#-------------------------------------------------------------------------------

def test_PRP(initial):
    """
    The 'expected initial states and solution pairs' below are provided
    as a sanity check, showing what the PlanRouteProblem soluton is
    expected to produce.  Provide the 'initial state' tuple as the
    argument to test_PRP, and the associate solution list of actions is
    expected as the result.
    The test assumes the goals are [(2,3),(3,2)], that the heuristic fn
    defined in PlanRouteProblem uses the manhattan_distance_with_heading()
    fn above, and the allowed locations are:
        [(0,0),(0,1),(0,2),(0,3),
        (1,0),(1,1),(1,2),(1,3),
        (2,0),            (2,3),
        (3,0),(3,1),(3,2),(3,3)]
    
    Expected intial state and solution pairs:
    (0,0,0) : ['Forward', 'Forward', 'Forward', 'TurnRight', 'Forward', 'Forward']
    (0,0,1) : ['TurnRight', 'Forward', 'Forward', 'Forward', 'TurnRight', 'Forward', 'Forward']
    (0,0,2) : ['TurnLeft', 'Forward', 'Forward', 'Forward', 'TurnLeft', 'Forward', 'Forward']
    (0,0,3) : ['Forward', 'Forward', 'Forward', 'TurnLeft', 'Forward', 'Forward']
    """
    return plan_route((initial[0],initial[1]), initial[2],
                      # Goals:
                      [(2,3),(3,2)],
                      # Allowed locations:
                      [(0,0),(0,1),(0,2),(0,3),
                       (1,0),(1,1),(1,2),(1,3),
                       (2,0),            (2,3),
                       (3,0),(3,1),(3,2),(3,3)])


#-------------------------------------------------------------------------------
# Plan Shot
#-------------------------------------------------------------------------------

def plan_shot(current, heading, goals, allowed):
    """ Plan route to nearest location with heading directed toward one of the
    possible wumpus locations (in goals), then append shoot action.
    NOTE: This assumes you can shoot through walls!!  That's ok for now. """
    if goals and allowed:
        psp = PlanShotProblem((current[0], current[1], heading), goals, allowed)
        node = search.astar_search(psp)
        if node:
            plan = node.solution()
            plan.append(action_shoot_str(None))
            # HACK:
            # since the wumpus_alive axiom asserts that a wumpus is no longer alive
            # when on the previous round we perceived a scream, we
            # need to enforce waiting so that itme elapses and knowledge of
            # "dead wumpus" can then be inferred...
            plan.append(action_wait_str(None))
            return plan

    # no route can be found, return empty list
    return []

#-------------------------------------------------------------------------------

class PlanShotProblem(search.Problem):
    def __init__(self, initial, goals, allowed):
        """ Problem defining planning to move to location to be ready to
              shoot at nearest wumpus location
        NOTE: Just like PlanRouteProblem, except goal is to plan path to
              nearest location with heading in direction of a possible
              wumpus location;
              Shoot and Wait actions is appended to this search solution
        Goal is generally a location (x,y) tuple, but state will be (x,y,heading) tuple
        initial = initial location, (x,y) tuple
        goals   = list of goal (x,y) tuples
        allowed = list of state (x,y) tuples that agent could move to """
        self.initial = initial # initial state
        self.goals = goals     # list of goals that can be achieved
        self.allowed = allowed # the states we can move into
        self.shootLoc()

    def shootingLoc(self):
        self.shootLoc = []  # finding loc to shoot wumpus
        for (xA, yA) in self.allowed:
            # 0='north', 1='west', 2='south', 3='east'
            for (x,y) in self.goals:
                if xA == x and yA < y:
                    self.shootLoc.append((xA, yA, 0))
                elif xA == x and yA > y:
                    self.shootLoc.append((xA, yA, 2))
                elif xA > x and yA == y:
                    self.shootLoc.append((xA, yA, 1))
                elif xA < x and yA < y:
                    self.shootLoc.append((xA, yA, 3))

    def h(self,node):
        """
        Heuristic that will be used by search.astar_search()
        """
        "*** YOUR CODE HERE ***"
        minD = 9999999
        for goal in self.shoot:
            minD = min(minD, manhattan_distance_with_heading(node.state, goal))
        return minD

    def actions(self, state):
        """
        Return list of allowed actions that can be made in state
        """
        "*** YOUR CODE HERE ***"
        x, y, direction = state[0:3]
        for (xN,yN,dirN) in self.shootLoc:
            if (x,y) is (xN,yN):
                return ['TurnLeft'] if dirN-direction is 1 else ['TurnRight']

        forward, right, left = state, state, state
        # 0='north', 1='west', 2='south', 3='east'
        if direction is 0:
            forward = (x, y + 1, 0)
            left = (x, y, 1)
            right = (x, y, 3)
        elif direction is 1:
            forward = (x - 1, y, 1)
            left = (x, y, 2)
            right = (x, y, 0)
        elif direction is 2:
            forward = (x, y - 1, 2)
            left = (x, y, 3)
            right = (x, y, 1)
        elif direction is 3:
            forward = (x + 1, y, 3)
            left = (x, y, 0)
            right = (x, y, 2)
        minF = 99999
        minL = 99999
        minR = 99999
        for goal in self.goals:
            dfwd = 99999
            allowedD = self.allowed
            if (forward[0], forward[1]) in allowedD:
                dfwd = manhattan_distance_with_heading(forward, goal)
            minF = min(minF, dfwd)
            minR = min(minR, manhattan_distance_with_heading(right, goal))
            minL = min(minL, manhattan_distance_with_heading(left, goal))
        if minF <= minR and minF <= minL:
            return ['Forward']
        elif minR <= minF and minR <= minL:
            return ['TurnRight']
        elif minL <= minF and minL <= minR:
            return ['TurnLeft']
        return None


    def result(self, state, action):
        """
        Return the new state after applying action to state
        """
        "*** YOUR CODE HERE ***"
        x, y, direction = state[0:3]
        # 0='north', 1='west', 2='south', 3='east'
        if proposition_bases_actions.index(action) is 0:
            if direction is 0:
                return x, y + 1, 0
            elif direction is 1:
                return x - 1, y, 1
            elif direction is 2:
                return x, y - 1, 2
            elif direction is 3:
                return x + 1, y, 3
        elif proposition_bases_actions.index(action) is 4:
            return (x, y, 0) if (direction + 1) > 3 else (x, y, direction + 1)
        elif proposition_bases_actions.index(action) is 5:
            return (x, y, 3) if (direction - 1) < 0 else (x, y, direction - 1)
        return state

    def goal_test(self, state):
        """
        Return True if state is a goal state
        """
        "*** YOUR CODE HERE ***"
        x, y = state[0:2]
        return True if (x, y) in self.shootLoc else False

#-------------------------------------------------------------------------------

def test_PSP(initial = (0,0,3)):
    """
    The 'expected initial states and solution pairs' below are provided
    as a sanity check, showing what the PlanShotProblem soluton is
    expected to produce.  Provide the 'initial state' tuple as the
    argumetn to test_PRP, and the associate solution list of actions is
    expected as the result.
    The test assumes the goals are [(2,3),(3,2)], that the heuristic fn
    defined in PlanShotProblem uses the manhattan_distance_with_heading()
    fn above, and the allowed locations are:
        [(0,0),(0,1),(0,2),(0,3),
        (1,0),(1,1),(1,2),(1,3),
        (2,0),            (2,3),
        (3,0),(3,1),(3,2),(3,3)]
    
    Expected intial state and solution pairs:
    (0,0,0) : ['Forward', 'Forward', 'TurnRight', 'Shoot', 'Wait']
    (0,0,1) : ['TurnRight', 'Forward', 'Forward', 'TurnRight', 'Shoot', 'Wait']
    (0,0,2) : ['TurnLeft', 'Forward', 'Forward', 'Forward', 'TurnLeft', 'Shoot', 'Wait']
    (0,0,3) : ['Forward', 'Forward', 'Forward', 'TurnLeft', 'Shoot', 'Wait']
    """
    return plan_shot((initial[0],initial[1]), initial[2],
                     # Goals:
                     [(2,3),(3,2)],
                     # Allowed locations:
                     [(0,0),(0,1),(0,2),(0,3),
                      (1,0),(1,1),(1,2),(1,3),
                      (2,0),            (2,3),
                      (3,0),(3,1),(3,2),(3,3)])
    
#-------------------------------------------------------------------------------
