from __future__ import annotations

from typing import TYPE_CHECKING

from algorithms import utils


if TYPE_CHECKING:
    from world.game_state import GameState


def evaluation_function(state: GameState) -> float:
    """
    Evaluation function for non-terminal states of the drone vs. hunters game.

    A good evaluation function can consider multiple factors, such as:
      (a) BFS distance from drone to nearest delivery point (closer is better).
          Uses actual path distance so walls and terrain are respected.
      (b) BFS distance from each hunter to the drone, traversing only normal
          terrain ('.' / ' ').  Hunters blocked by mountains, fog, or storms
          are treated as unreachable (distance = inf) and pose no threat.
      (c) BFS distance to a "safe" position (i.e., a position that is not in the path of any hunter).
      (d) Number of pending deliveries (fewer is better).
      (e) Current score (higher is better).
      (f) Delivery urgency: reward the drone for being close to a delivery it can
          reach strictly before any hunter, so it commits to nearby pickups
          rather than oscillating in place out of excessive hunter fear.
      (g) Adding a revisit penalty can help prevent the drone from getting stuck in cycles.

    Returns a value in [-1000, +1000].

    Tips:
    - Use state.get_drone_position() to get the drone's current (x, y) position.
    - Use state.get_hunter_positions() to get the list of hunter (x, y) positions.
    - Use state.get_pending_deliveries() to get the set of pending delivery (x, y) positions.
    - Use state.get_score() to get the current game score.
    - Use state.get_layout() to get the current layout.
    - Use state.is_win() and state.is_lose() to check terminal states.
    - Use bfs_distance(layout, start, goal, hunter_restricted) from algorithms.utils
      for cached BFS distances. hunter_restricted=True for hunter-only terrain.
    - Use dijkstra(layout, start, goal) from algorithms.utils for cached
      terrain-weighted shortest paths, returning (cost, path).
    - Consider edge cases: no pending deliveries, no hunters nearby.
    - A good evaluation function balances delivery progress with hunter avoidance.
    """
    if state.is_win():
        return 1000.0

    if state.is_lose():
        return -1000.0

    drone_pos = state.get_drone_position()
    hunter_positions = state.get_hunter_positions()
    pending_deliveries = state.get_pending_deliveries()
    score = state.get_score()
    layout = state.get_layout()

    value = float(score)

    # penalizar entregas pendientes
    value -= 180.0 * len(pending_deliveries)

    # recompensa por acercarse a la entrega mas cercana (costo de terreno real)
    if pending_deliveries:
        delivery_costs = []
        for delivery in pending_deliveries:
            cost, _ = utils.dijkstra(layout, drone_pos, delivery)
            if cost != float("inf"):
                delivery_costs.append(cost)

        if delivery_costs:
            closest_delivery_cost = min(delivery_costs)
            value -= 20.0 * closest_delivery_cost
        else:
            value -= 250.0

    # riesgo por cercania del cazador (BFS restringido a terreno libre)
    if hunter_positions:
        hunter_dists = []
        for hunter in hunter_positions:
            dist = utils.bfs_distance(layout, hunter, drone_pos, hunter_restricted=True)
            if dist != float("inf"):
                hunter_dists.append(dist)

        if hunter_dists:
            closest_hunter = min(hunter_dists)

            if closest_hunter == 0:
                return -1000.0
            elif closest_hunter == 1:
                value -= 350.0
            elif closest_hunter == 2:
                value -= 120.0
            elif closest_hunter == 3:
                value -= 40.0
            else:
                value += 10.0 * closest_hunter

    # bonus por entrega alcanzable antes que cualquier cazador
    for delivery in pending_deliveries:
        drone_cost, _ = utils.dijkstra(layout, drone_pos, delivery)

        hunter_best = float("inf")
        for hunter in hunter_positions:
            hdist = utils.bfs_distance(layout, hunter, delivery, hunter_restricted=True)
            hunter_best = min(hunter_best, hdist)

        if drone_cost < hunter_best:
            value += 90.0

    return value
