from __future__ import annotations
from os import path

from planning.pddl import Action, Problem, apply_action, is_applicable
import itertools
from planning.utils import Queue


# ---------------------------------------------------------------------------
# HTN Infrastructure
# ---------------------------------------------------------------------------


class HLA:
    """
    A High-Level Action (HLA) in HTN planning.

    An HLA is an abstract task that can be refined into sequences of
    more primitive actions (or other HLAs). Each refinement is a list
    of HLA or Action objects.

    name:        Human-readable name for display
    refinements: List of possible refinements, each a list of HLA/Action objects
    """

    def __init__(self, name: str, refinements: list[list] | None = None) -> None:
        self.name = name
        self.refinements = refinements or []

    def __repr__(self) -> str:
        return f"HLA({self.name})"


def is_primitive(action: Action | HLA) -> bool:
    """Return True if action is a primitive (grounded Action), False if it is an HLA."""
    return isinstance(action, Action)


def is_plan_primitive(plan: list[Action | HLA]) -> bool:
    """Return True if every step in the plan is a primitive action."""
    return all(is_primitive(step) for step in plan)


# ---------------------------------------------------------------------------
# Punto 5a – hierarchicalSearch
# ---------------------------------------------------------------------------


def hierarchicalSearch(problem: Problem, hlas: list[HLA]) -> list[Action]:

    queue = Queue()
    queue.push(hlas)

    while not queue.isEmpty():

        current_plan = queue.pop()

        # -------------------------------------------------
        # Simular prefijo primitivo
        # -------------------------------------------------

        state = problem.initial_state
        valid_prefix = True

        for step in current_plan:

            if not is_primitive(step):
                break

            if is_applicable(state, step):
                state = apply_action(state, step)
            else:
                valid_prefix = False
                break

        if not valid_prefix:
            continue

        # -------------------------------------------------
        # Si el plan ya es completamente primitivo
        # -------------------------------------------------

        if is_plan_primitive(current_plan):

            if problem.isGoalState(state):
                return current_plan

        else:

            # Expandir primer HLA encontrado
            for i, step in enumerate(current_plan):

                if not is_primitive(step):

                    for refinement in step.refinements:

                        new_plan = (
                            current_plan[:i]
                            + refinement
                            + current_plan[i+1:]
                        )

                        queue.push(new_plan)

                    break

    return []
    ### End of your code ###


# ---------------------------------------------------------------------------
# Punto 5b – HLA Definitions
# ---------------------------------------------------------------------------


def build_htn_hierarchy(problem: Problem) -> list[HLA]:
    """
    Build HTN HLAs for the rescue domain.

    The hierarchy defines four HLA types:
      - Navigate(from, to):       Move the robot step by step from one cell to another
      - PrepareSupplies(s, m):    Collect supplies and set them up at the medical post
      - ExtractPatient(p, m):     Pick up the patient and bring them to the medical post
      - FullRescueMission(s,p,m): Complete one rescue: prepare supplies + extract + rescue

    Refinements are built from the ground state to generate concrete Action objects.

    Tip: Refinements for Navigate are all single-step Move sequences between
         adjacent cells. PrepareSupplies and ExtractPatient chain Navigate HLAs
         with primitive PickUp, SetupSupplies, PutDown, and Rescue actions.
    """
    ### Your code here ###
    objects = problem.objects

    robot = objects["robots"][0]
    medical_post = objects["medical_posts"][0]

    patients = objects["patients"]
    supplies = objects["supplies"]

    initial_state = problem.initial_state

    # -------------------------------------------------
    # Helpers
    # -------------------------------------------------

    def find_location(obj):
        for fluent in initial_state:
            if fluent[0] == "At" and fluent[1] == obj:
                return fluent[2]
        return None

    # -------------------------------------------------
    # Navigate HLA
    # -------------------------------------------------

    def build_navigate(from_cell, to_cell):

        if from_cell == to_cell:
            return HLA(
                f"Navigate({from_cell}->{to_cell})",
                refinements=[[]]
            )

        # -------------------------------------------------
        # Construir grafo de adyacencia
        # -------------------------------------------------

        adjacency = {}

        for fluent in initial_state:

            if fluent[0] == "Adjacent":

                a = fluent[1]
                b = fluent[2]

                if a not in adjacency:
                    adjacency[a] = []

                adjacency[a].append(b)

        # -------------------------------------------------
        # BFS para encontrar camino
        # -------------------------------------------------

        frontier = Queue()
        frontier.push((from_cell, [from_cell]))

        visited = set()

        path = None

        while not frontier.isEmpty():

            current, current_path = frontier.pop()

            if current == to_cell:
                path = current_path
                break

            if current in visited:
                continue

            visited.add(current)

            for neighbor in adjacency.get(current, []):

                if neighbor not in visited:
                    frontier.push(
                        (neighbor, current_path + [neighbor])
                    )

    # -------------------------------------------------
    # Si no existe camino
    # -------------------------------------------------
        print("NAVIGATE")
        print("FROM:", from_cell)
        print("TO:", to_cell)
        print("PATH:", path)
        if path is None:
            return HLA(
                f"Navigate({from_cell}->{to_cell})",
                refinements=[]
            )

    # -------------------------------------------------
    # Convertir path a acciones Move
    # -------------------------------------------------

        moves = []

        for i in range(len(path) - 1):

            a = path[i]
            b = path[i + 1]

            move_action = Action(
                name=f"Move({robot},{a},{b})",

                precond_pos=[
                    ("At", robot, a),
                    ("Adjacent", a, b),
                    ("Free", b),
                ],

                precond_neg=[],

                add_list=[
                    ("At", robot, b),
                    ("Free", a),
                ],

                del_list=[
                    ("At", robot, a),
                    ("Free", b),
                ],
            )

            moves.append(move_action)

        return HLA(
            f"Navigate({from_cell}->{to_cell})",
            refinements=[moves]
        )

    # -------------------------------------------------
    # Build missions
    # -------------------------------------------------

    robot_loc = find_location(robot)

    supply = supplies[0]
    supply_loc = find_location(supply)

    pickup_supply = Action(
        f"PickUp({robot},{supply},{supply_loc})",

        precond_pos=[
            ("At", robot, supply_loc),
            ("At", supply, supply_loc),
            ("HandsFree", robot),
            ("Pickable", supply),
        ],

        precond_neg=[],

        add_list=[
            ("Holding", robot, supply),
        ],

        del_list=[
            ("At", supply, supply_loc),
            ("HandsFree", robot),
        ],
    )

    setup_supply = Action(
        f"SetupSupplies({robot},{supply},{medical_post})",

        precond_pos=[
            ("At", robot, medical_post),
            ("MedicalPost", medical_post),
            ("Holding", robot, supply),
        ],

        precond_neg=[],

        add_list=[
            ("SuppliesReady", medical_post),
            ("HandsFree", robot),
        ],

        del_list=[
            ("Holding", robot, supply),
        ],
    )

    prepare_supplies = HLA(
        "PrepareSupplies",

        refinements=[[
            build_navigate(robot_loc, supply_loc),
            pickup_supply,
            build_navigate(supply_loc, medical_post),
            setup_supply,
        ]]
    )

    # -------------------------------------------------
    # Misiones
    # -------------------------------------------------

    missions = [prepare_supplies]

    current_robot_loc = medical_post

    for patient in patients:

        patient_loc = find_location(patient)

        pickup_patient = Action(
            f"PickUp({robot},{patient},{patient_loc})",

            precond_pos=[
                ("At", robot, patient_loc),
                ("At", patient, patient_loc),
                ("HandsFree", robot),
                ("Pickable", patient),
            ],

            precond_neg=[],

            add_list=[
                ("Holding", robot, patient),
            ],

            del_list=[
                ("At", patient, patient_loc),
                ("HandsFree", robot),
            ],
        )

        putdown_patient = Action(
            f"PutDown({robot},{patient},{medical_post})",

            precond_pos=[
                ("At", robot, medical_post),
                ("Holding", robot, patient),
            ],

            precond_neg=[],

            add_list=[
                ("At", patient, medical_post),
                ("HandsFree", robot),
            ],

            del_list=[
                ("Holding", robot, patient),
            ],
        )

        rescue_patient = Action(
            f"Rescue({robot},{patient},{medical_post})",

            precond_pos=[
                ("At", robot, medical_post),
                ("At", patient, medical_post),
                ("MedicalPost", medical_post),
                ("SuppliesReady", medical_post),
            ],

            precond_neg=[],

            add_list=[
                ("Rescued", patient),
            ],

            del_list=[
                ("At", patient, medical_post),
            ],
        )

        extract_patient = HLA(
            f"ExtractPatient({patient})",

            refinements=[[
                build_navigate(current_robot_loc, patient_loc),
                pickup_patient,
                build_navigate(patient_loc, medical_post),
                putdown_patient,
                rescue_patient,
            ]]
        )

        missions.append(extract_patient)

        current_robot_loc = medical_post

    # -------------------------------------------------
    # TOP LEVEL HLA
    # -------------------------------------------------

    top_level = HLA(
        "CompleteMission",
        refinements=[missions]
    )

    return [top_level]
    ### End of your code ###
