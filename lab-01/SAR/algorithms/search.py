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
        if problem.isGoalState(currNode):
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
    queue = utils.Queue()
    startNode = problem.getStartState()
    initialNeighbors = problem.getSuccessors(problem.getStartState())
    queue.push([startNode, initialNeighbors])  
    visited = set()
    
    # this will keep track of the parent node to reconstruct the path 
    parentMap = {}
    parentMap[startNode] = None

    while not queue.isEmpty():
        currState = queue.pop()
        currNode = currState[0]
        neighbors = currState[1]

        if problem.isGoalState(currNode):
            goalNode = currNode
            return _build_bfs_path(parentMap, startNode, goalNode)
        
        # for all neighbors of the current node, if not visited, add to queue and mark as visited
        for neighbor in neighbors: 
            node = neighbor[0]
            direction = neighbor[1]

            if node not in visited:
                visited.add(node)

                neighbors = problem.getSuccessors(node)
                queue.push([node,neighbors])
                parentMap[node] = (currNode, direction)
    return -1


def _build_bfs_path(parentMap, startNode, goalNode):
    path = []
    currentNode = goalNode 
     
    while currentNode != startNode:
        parent = parentMap[currentNode][0]
        direction = parentMap[currentNode][1]

        path.append(direction)
        currentNode = parent
    # python trick to reverse list since we started backwards
    return path[::-1]


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
