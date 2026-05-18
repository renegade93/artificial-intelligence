from __future__ import annotations

from planning.pddl import (
    ActionSchema,
    State,
    Objects,
    get_all_groundings,
    get_applicable_actions,
)


def nullHeuristic(
    state: State,
    goal: State,
    domain: list[ActionSchema],
    objects: Objects,
) -> float:
    """Trivial heuristic — always returns 0 (equivalent to uniform-cost search)."""
    return 0


# ---------------------------------------------------------------------------
# Cache de groundings — evita re-generar todos los Action grounded
# en cada llamada de la heurística (A* la llama miles de veces).
# Se invalida con la identidad de (domain, objects), que para un mismo
# Problem permanece estable durante toda la búsqueda.
# ---------------------------------------------------------------------------

_GROUNDINGS_CACHE: dict[tuple[int, int], list] = {}


def _cached_groundings(domain: list[ActionSchema], objects: Objects) -> list:
    key = (id(domain), id(objects))
    if key not in _GROUNDINGS_CACHE:
        _GROUNDINGS_CACHE[key] = get_all_groundings(domain, objects)
    return _GROUNDINGS_CACHE[key]


# ---------------------------------------------------------------------------
# Punto 4a – Ignore-Preconditions Heuristic
# ---------------------------------------------------------------------------


def ignorePreconditionsHeuristic(
    state: State,
    goal: State,
    domain: list[ActionSchema],
    objects: Objects,
) -> float:
    """
    Estimate the number of actions needed to satisfy all goal fluents,
    ignoring all action preconditions.

    With no preconditions, any action can be applied at any time.
    Each action can satisfy all goal fluents in its add_list in one step.
    The minimum number of actions to cover all unsatisfied goal fluents is
    a lower bound on the true plan length → this heuristic is admissible.

    Algorithm (greedy set cover):
      1. Compute unsatisfied = goal − state  (fluents still needed).
      2. Ground all actions ignoring preconditions and collect their add_lists.
      3. Greedily pick the action whose add_list covers the most unsatisfied fluents.
      4. Repeat until all fluents are covered; count the actions used.

    Tip: frozenset supports set difference (-) and intersection (&).
         You only need to ground actions once per call (use get_applicable_actions
         with the initial state, or generate all groundings regardless of state).
         Remember: with no preconditions, every grounding is "applicable".
    """
    ### Your code here ###
    unsatisfied = goal - state
    if not unsatisfied:
        return 0

    # Sin precondiciones, todo grounding es candidato.
    all_actions = _cached_groundings(domain, objects)

    count = 0
    while unsatisfied:
        # Pick action cuyo add_list cubra más fluentes pendientes.
        best_cover_size = 0
        best_cover: frozenset = frozenset()
        for action in all_actions:
            cover = action.add_list & unsatisfied
            if len(cover) > best_cover_size:
                best_cover_size = len(cover)
                best_cover = cover

        if best_cover_size == 0:
            # Ningún add_list aporta a los pendientes → objetivo inalcanzable.
            return float("inf")

        unsatisfied = unsatisfied - best_cover
        count += 1

    return count
    ### End of your code ###


# ---------------------------------------------------------------------------
# Punto 4b – Ignore-Delete-Lists Heuristic
# ---------------------------------------------------------------------------


def ignoreDeleteListsHeuristic(
    state: State,
    goal: State,
    domain: list[ActionSchema],
    objects: Objects,
) -> float:
    """
    Estimate the plan cost by solving a relaxed problem where no action
    has a delete list (effects never remove fluents from the state).

    In this monotone relaxation, the state only grows over time (fluents are
    never removed), so hill-climbing always makes progress and cannot loop.

    Algorithm (hill-climbing on the relaxed problem):
      1. Start from the current state with a relaxed (monotone) apply function.
      2. At each step, pick the grounded action that adds the most unsatisfied
         goal fluents (greedy hill-climbing).
      3. Count steps until all goal fluents are satisfied (or until no progress).

    Tip: In the relaxed problem, apply_action never removes fluents.
         You can implement this by treating del_list as empty for all actions.
         Use get_applicable_actions to enumerate applicable grounded actions at
         each step (preconditions still apply in the relaxed model).
    """
    ### Your code here ###
    if goal.issubset(state):
        return 0

    all_actions = _cached_groundings(domain, objects)

    current = set(state)
    steps = 0
    # Cota de seguridad: el problema relajado es monótono, en n iteraciones
    # con n = |fluentes totales| se alcanza el punto fijo.
    max_iterations = 10_000

    while not goal.issubset(current):
        unsatisfied = goal - current

        # Acción aplicable que más fluentes pendientes agrega.
        best_action = None
        best_added = 0
        # En el problema relajado las precondiciones siguen evaluándose,
        # pero como nunca borramos nada, una acción aplicada una vez nunca
        # vuelve a serlo "necesaria" sobre los mismos fluentes nuevos.
        for action in all_actions:
            if not action.precond_pos.issubset(current):
                continue
            if not action.precond_neg.isdisjoint(current):
                continue
            new_in_goal = (action.add_list & unsatisfied)
            if len(new_in_goal) > best_added:
                best_added = len(new_in_goal)
                best_action = action

        if best_action is None:
            # No hay acción aplicable que contribuya al objetivo.
            # Buscamos cualquier acción aplicable que agregue fluentes nuevos,
            # para habilitar futuras acciones (relajación monótona).
            for action in all_actions:
                if not action.precond_pos.issubset(current):
                    continue
                if not action.precond_neg.isdisjoint(current):
                    continue
                new_fluents = action.add_list - current
                if new_fluents:
                    best_action = action
                    break

            if best_action is None:
                return float("inf")

        # Aplicar en modo relajado: solo agregar, nunca borrar.
        current = current | set(best_action.add_list)
        steps += 1

        if steps > max_iterations:
            return float("inf")

    return steps
    ### End of your code ###
