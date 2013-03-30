'''An A-Star search for solving rolling die maze puzzles.

Author: Piyush Verma

Provice the Test file path as a command line argument'''

from heapq import heappush, heappop
from sys import argv, exit

# A list of symbols contained in the puzzle i.e.(SG.*).
puzzleList = []

# A map of visited states sorted according to their cost.
visitedStates = {}

# The goal location (x, y).
goal_location = None



# A list of all possible States.
fringe = []

# Global counters for number of states visitedStates and generated.
numberOfVisited = 0
numberOfGenerated = 0

# The heuristic function
heuristic = lambda state: 0

"""Prints a puzzle configuration which is extracted from the puzzle file."""
def print_puzzle(other=None):
    p = other if other else puzzleList
    for y in range(len(p[0])):
        print("".join([str(p[x][y]) for x in range(len(p))]))
   


"""Tests whether the current (x,y) coordinate in the puzzle is a space(.) and not an obstacle(*)."""
def valid_space(x, y):
    return (0 <= x < len(puzzleList) and
        0 <= y < len(puzzleList[0]) and
        puzzleList[x][y] != '*')

class State(object):
    """Encapsulates the variables of the problem state into a single object"""
    
    def __init__(self, x, y, die, action):
        self.x = x
        self.y = y
        self.die = die
        self.action = action
    
    @staticmethod
    def isValid(dieState):
        """Checks if the current dieState is valid or not. 
        Returns false if the dieState is off the board configurations,
        on an obstacle (*) or the 6 side of the die is up."""
        
        return valid_space(dieState.x, dieState.y) and dieState.die[0] != 6
    
    def getNeighbor(self):
        """Returns all the neighboring nodes of the current state"""
        # Die faces: (upwards, northwards, eastwards)
        (up, north, east) = self.die
        return filter(State.isValid, [
            State(self.x, self.y + 1, (north, 7 - up, east), "S"), # South neighbor state
            State(self.x, self.y - 1, (7 - north, up, east), "N"), # North neighbor state
            State(self.x + 1, self.y, (7 - east, north, up), "E"), # East neighbor state
            State(self.x - 1, self.y, (east, north, 7 - up), "W"), # West neighbor state
        ])
    
    def __eq__(self, o):
        """function for check if self == o"""
        return isinstance(o, State) and self.x == o.x and self.y == o.y and self.die == o.die
    
    def __hash__(self):
        """method to get the hash value of the object"""
        return hash((self.x, self.y, self.die))
    
    '''method to stringize the state object'''
    def __str__(self):
        return "<%s, %s %s>" % (self.x, self.y, self.die)
    __repr__ = __str__
    
    '''returns the current die state'''
    def getDie(self):
        return self.die
    
    '''returns the current x coordinate in the puzzle'''
    def getX(self):
        return self.x
    
    '''returns the current y coordinate in the puzzle'''
    def getY(self):
        return self.y
    
class Node(object):
    """A node object for the search algorithm"""
    
    def __init__(self, state, cost, parent):
        self.state = state
        self.cost = cost
        self.parent = parent
    
    def expandNode(self):
        """Method to expand the current node and push its neighbors to the fringe."""
        global numberOfGenerated, numberOfVisited
        visitedStates[self.state] = self.cost
        numberOfVisited += 1
        for neighborNode in self.state.getNeighbor():
            nextCost = self.cost + 1 + heuristic(neighborNode)
            if neighborNode not in visitedStates or nextCost < visitedStates[neighborNode]:
                heappush(fringe, Node(neighborNode, nextCost, self))
                numberOfGenerated += 1
    
    def unwind(self):
        """Backtrack the path from the start node to the current node."""
        if self.parent == None:
            return []
        else:
            path = self.parent.unwind()
            path.append((self.state.action, self.state))
            return path
    
    def is_outdated(self):
        """Check if a better cost has been found to reach the node."""
        return self.state in visitedStates and visitedStates[self.state] < self.cost
    
    def __lt__(self, other):
        """Compare method for sorting."""
        return self.cost < other.cost
    
    def __str__(self):
        return "%s | %s" % (self.state, self.cost)
    __repr__ = __str__
    

def a_star_search(start):
    """Perform A-star search on the puzzle using the current heuristic and return a path to the goal location if one exists."""
    global numberOfVisited, numberOfGenerated, goal_location, fringe, visitedStates
    numberOfVisited = 1
    numberOfGenerated = 0
    fringe = []
    visitedStates = {}
    def checkGoal(state):
        return (state.x, state.y) == goal_location and state.die[0] == 1
    fringe.append(Node(start, 0, None))
    visitedStates[start] = 0
    while True:
        if len(fringe) == 0:
            return ["FAIL"], numberOfVisited, numberOfGenerated
        node = None
        # Loop to skip visited nodes.
        while not node or node.is_outdated():
            node = heappop(fringe)
        if checkGoal(node.state):
            return node.unwind(), numberOfVisited, numberOfGenerated
        else:
            node.expandNode()

# Heuristics definition
#Definition of Euclidean Distance
def euclideanDistance(state):
    return ((state.x - goal_location[0]) ** 2 + (state.y - goal_location[1]) ** 2) ** 0.5

#Definition of Manhattan Distance
def manhattanDistance(state):
    return abs(state.x - goal_location[0]) + abs(state.y - goal_location[1])

# Path distances for the path heuristic.
distances = []

def generate_distances():
    """A breadth first search to find distances from the goal for each loc."""
    global distances
    # Copy the puzzleList.
    distances = list(map(list, puzzleList))
    def unseen(v):
        x, y = v
        return valid_space(x, y) and type(distances[x][y]) != int
    def next_states(x, y):
        return filter(unseen, [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)])
    fringe = [(goal_location[0], goal_location[1])]
    d = 0
    while len(fringe) > 0:
        layer = list(fringe)
        fringe = set([])
        for x, y in layer:
            distances[x][y] = d
            fringe.update(next_states(x, y))
        d += 1

def pathDistance(state):
    return distances[state.x][state.y]

def main():
    if len(argv) < 2:
        print("Usage: die_a_star.py puzzle_file")
        exit(1)
    
    global puzzleList, goal_location, heuristic
    start = None
    print "Puzzle extracted from the text file is:"
    with open(argv[1], 'r') as f:
        lines = f.read().splitlines()
        #lines.reverse()
        
    
    # Process the input puzzle text file and locate the start and the goal states.
    puzzleList = [[] for _ in range(len(lines[0]))]
    for y, line in enumerate(lines):
        for x, c in enumerate(line):
            puzzleList[x].append(c)
            if c not in '.*SG':
                print("%s is an invalid character for the current format of the puzzles." % c)
                exit(1)
            if c == 'S':
                if start != None:
                    print("There cannot be more than 1 start states! ")
                    exit(1)
                start = State(x, y, (1, 2, 3), None)
            if c == 'G':
                if goal_location != None:
                    print("There cannot be more than 1 goal states! ")
                    exit(1)
                goal_location = (x, y)
    if start == None:
        print("Start state is absent! ")
        exit(1)
    if goal_location == None:
        print("Goal state is absent! ")
        exit(1)
    
    print_puzzle(puzzleList)
    def print_results():
        
        path, nvisited, ngenerated = a_star_search(start)
        
            
        if(not path.__eq__(["FAIL"])):
            for action, new_state in path:
                if(action is 'N'):
                    print ("Moving North : ")
                    print ("Coordinates : %s, %s" %(new_state.getX(), new_state.getY()))
                    print ("Die States (Up, North, East) : %s, %s, %s" % new_state.getDie())
                elif(action is 'S'):
                    print ("Moving South : ")
                    print ("Coordinates : %s, %s" %(new_state.getX(), new_state.getY()))
                    print ("Die States (Up, North, East) : %s, %s, %s" % new_state.getDie())
                    
                elif(action is 'E'):
                    print ("Moving East : ")
                    print ("Coordinates : %s, %s" %(new_state.getX(), new_state.getY()))
                    print ("Die States (Up, North, East) : %s, %s, %s" % new_state.getDie())
                    
                else:
                    print ("Moving West : ")
                    print ("Coordinates : %s, %s" %(new_state.getX(), new_state.getY()))
                    print ("Die States (Up, North, East) : %s, %s, %s" % new_state.getDie())
                    
            print("\n%s states were visited and %s were generated.\n" % (nvisited, ngenerated))
        else:
            print "No solution Found.\n"
            print("%s states were visited and %s were generated.\n" % (nvisited, ngenerated))
    print("Heuristic: Euclidean distance\nSolution:\n")
    heuristic = euclideanDistance
    print_results()
    print("Heuristic: Manhattan distance\nSolution:\n")
    heuristic = manhattanDistance
    print_results()
    print("Heuristic: Path distance\nSolution:\n")
    generate_distances()
    heuristic = pathDistance
    print_results()

if __name__ == "__main__":
    main()