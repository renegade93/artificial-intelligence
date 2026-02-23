import math
from typing import Any, Tuple
from algorithms import utils
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.
    """
    return _manhattan(state, problem.goal)


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    """
    survivor = problem.goal
    dx = survivor[0] - state[0]
    dy = survivor[1] - state[1]

    return math.sqrt(dx**2 + dy**2)

def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
    """
    Your heuristic for the MultiSurvivorProblem.

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider: distance to nearest survivor + MST of remaining survivors
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    startNode = state[0]
    survivorsGrid = state[1]
    survivors = survivorsGrid.asList(key=True)
    
    if len(survivors) == 0:
        return 0

    survivorsCopy = survivors.copy()
    survivorsCopy.append(startNode)
    
    return _primMSTWeight(survivorsCopy)

# DISCLAIMER: most of this method was written with the aid of Claude
# (I asked it to generate seudo code for PRIM's algorithm, and then translated that into code)
def _primMSTWeight(nodes):
    INFINITY = float('inf')

    if len(nodes) == 1:
        return 0

    visited = {nodes[0]}
    total_cost = 0

    while len(visited) < len(nodes):
        best_cost = INFINITY
        best_node = None

        for x in visited:
            for y in nodes:
                if y not in visited:
                    dist = _manhattan(x, y)
                    if dist < best_cost:
                        best_cost = dist
                        best_node = y

        total_cost += best_cost
        visited.add(best_node)

    return total_cost

def _manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])