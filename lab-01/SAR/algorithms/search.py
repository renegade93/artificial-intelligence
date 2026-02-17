from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    """
    # initialize dfs
    stack = utils.Stack()
    startState = problem.getStartState()
    initialNeighbors = problem.getSuccessors(problem.getStartState())
    stack.push([startState, initialNeighbors, None ])  
    visited = []
    path = []

    while not stack.isEmpty():
        currState = stack.peek()
        currNode = currState[0]
        currNeighbors = currState[1]

        # check if current node is the goal
        if problem.isGoalState(currState[0]):
            print(stack.list)
            return path
        
        # visit node
        visited.append(currNode)

        # check last neighbor of current node 
        if not len(currNeighbors) == 0:
            lastNeighbor = currNeighbors.pop()
            lastNeighborCoord = lastNeighbor[0]
            lastNeighborAction = lastNeighbor[1]

            # if last neighbor not visited, add to the stack
            if lastNeighborCoord not in visited:
                nextSuccesors = problem.getSuccessors(lastNeighborCoord)
                stack.push([lastNeighborCoord, nextSuccesors, lastNeighborAction])
                path.append(lastNeighborAction)
        else:
            # if dead end, pop stack
            stack.pop()
            path.pop()
    return -1



def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    # TODO: Add your code here
    utils.raiseNotDefined()


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """

    # TODO: Add your code here
    utils.raiseNotDefined()


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    # TODO: Add your code here
    utils.raiseNotDefined()


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
