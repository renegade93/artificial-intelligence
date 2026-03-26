from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from algorithms.problems_csp import DroneAssignmentCSP


def backtracking_search(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Basic backtracking search without optimizations.

    Tips:
    - An assignment is a dictionary mapping variables to values (e.g. {X1: Cell(1,2), X2: Cell(3,4)}).
    - Use csp.assign(var, value, assignment) to assign a value to a variable.
    - Use csp.unassign(var, assignment) to unassign a variable.
    - Use csp.is_consistent(var, value, assignment) to check if an assignment is consistent with the constraints.
    - Use csp.is_complete(assignment) to check if the assignment is complete (all variables assigned).
    - Use csp.get_unassigned_variables(assignment) to get a list of unassigned variables.
    - Use csp.domains[var] to get the list of possible values for a variable.
    - Use csp.get_neighbors(var) to get the list of variables that share a constraint with var.
    - Add logs to measure how good your implementation is (e.g. number of assignments, backtracks).

    You can find inspiration in the textbook's pseudocode:
    Artificial Intelligence: A Modern Approach (4th Edition) by Russell and Norvig, Chapter 5: Constraint Satisfaction Problems
    """

    def backtrack(assignment: dict[str, str], depth = 0) -> dict[str, str] | None:
        # to visualize recursion
        spaces = "  " * depth
        print(f"{spaces}backtrack(depth={depth}, assigned={list(assignment.items())})")

        # check if assignment is complete
        if csp.is_complete(assignment):
            print(f"Assignments completed: {len(assignment)} ✅")
            print(f"{spaces}Complete assignment found: {assignment}")
            return assignment
        
        # grab a variable that isn't assigned yet (order wont't matter in this case)
        unassigned_vars = csp.get_unassigned_variables(assignment)
        var = unassigned_vars[0]

        # get list of possible values for a variable
        possible_vals = csp.domains[var]

        for val in possible_vals:
            print(f"{spaces}Trying out {val}")
            # check if value is consistent and assign it
            if csp.is_consistent(var, val, assignment):
                print(f"{spaces}  Looking at {var} = {val}")
                csp.assign(var, val, assignment)

                # recursive call!
                result = backtrack(assignment)   

                if result is not None:
                    return result
                else:
                    # make sure to unassign if that value didn't work out
                    print(f"{spaces}  {var} = {val} failed, backtracking")
                    csp.unassign(var, assignment)
        return None                 
    return backtrack({})


def backtracking_fc(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking search with Forward Checking.

    Tips:
    - Forward checking: After assigning a value to a variable, eliminate inconsistent values from
      the domains of unassigned neighbors. If any neighbor's domain becomes empty, backtrack immediately.
    - Save domains before forward checking so you can restore them on backtrack.
    - Use csp.get_neighbors(var) to get variables that share constraints with var.
    - Use csp.is_consistent(neighbor, val, assignment) to check if a value is still consistent.
    - Forward checking reduces the search space by detecting failures earlier than basic backtracking.
    """
    def backtrack(assignment: dict[str, str], depth = 0) -> dict[str, str] | None:
        # to visualize recursion
        spaces = "  " * depth
        print(f"{spaces}backtrack(depth={depth}, assigned={list(assignment.items())})")

        # check if assignment is complete
        if csp.is_complete(assignment):
            print(f"Assignments completed: {len(assignment)} ✅")
            print(f"{spaces}Complete assignment found: {assignment}")
            return assignment
        
        # grab a variable that isn't assigned yet (order wont't matter in this case)
        unassigned_vars = csp.get_unassigned_variables(assignment)
        var = unassigned_vars[0]

        # get list of possible values for a variable
        possible_vals = csp.domains[var]

        for val in possible_vals:
            print(f"{spaces}Trying out {val}")
            # check if value is consistent and assign it
            if csp.is_consistent(var, val, assignment):
                print(f"{spaces}  Looking at {var} = {val}")
                csp.assign(var, val, assignment)

                # preserve domaains before forward checking to restore them later
                preserved_domains = {}
                for domain in csp.domains:
                    preserved_domains[domain] = list(csp.domains[domain])

                # flag used to check if the domain of any neighbor went empty
                is_empty_domain = False
                for neighbor in csp.get_neighbors(var):
                    # make sure it hasn't been assigned yet
                    if neighbor not in assignment:
                        for neighbor_val in list(csp.domains[neighbor]):
                            if not csp.is_consistent(neighbor, neighbor_val, assignment):
                                # remove if inconsistent with the current assignment
                                csp.domains[neighbor].remove(neighbor_val)
                        # check if empty
                        if len(csp.domains[neighbor]) == 0:
                            is_empty_domain = True

                            # this break is necessary to stop forwaard checking 
                            break
                        
                if is_empty_domain is not True:
                      result = backtrack(assignment, depth + 1)
                      if result is not None:
                          return result                
                 
                csp.domains = preserved_domains
                print(f"{spaces}  {var} = {val} failed, backtracking")
                csp.unassign(var, assignment)
        return None
    return backtrack({})


def backtracking_ac3(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking search with AC-3 arc consistency.

    Tips:
    - AC-3 enforces arc consistency: for every pair of constrained variables (Xi, Xj), every value
      in Xi's domain must have at least one supporting value in Xj's domain.
    - Run AC-3 before starting backtracking to reduce domains globally.
    - After each assignment, run AC-3 on arcs involving the assigned variable's neighbors.
    - If AC-3 empties any domain, the current assignment is inconsistent - backtrack.
    - You can create helper functions such as:
      - a values_compatible function to check if two variable-value pairs are consistent with the constraints.
      - a revise function that removes unsupported values from one variable's domain.
      - an ac3 function that manages the queue of arcs to check and calls revise.
      - a backtrack function that integrates AC-3 into the search process.
    """
    def backtrack(assignment: dict[str, str], depth = 0) -> dict[str, str] | None:
        # to visualize recursion
        spaces = "  " * depth
        print(f"{spaces}backtrack(depth={depth}, assigned={list(assignment.items())})")

        # check if assignment is complete
        if csp.is_complete(assignment):
            print(f"Assignments completed: {len(assignment)} ✅")
            print(f"{spaces}Complete assignment found: {assignment}")
            return assignment
        
        # grab a variable that isn't assigned yet (order wont't matter in this case)
        unassigned_vars = csp.get_unassigned_variables(assignment)
        var = unassigned_vars[0]

        # get list of possible values for a variable
        possible_vals = csp.domains[var]

        for val in possible_vals:
            print(f"{spaces}Trying out {val}")
            # check if value is consistent and assign it
            if csp.is_consistent(var, val, assignment):
                print(f"{spaces}  Looking at {var} = {val}")
                csp.assign(var, val, assignment)

                # preserve domaains before forward checking to restore them later
                preserved_domains = {}
                for domain in csp.domains:
                    preserved_domains[domain] = list(csp.domains[domain])
               
                    arcs = [(neighbor, var) for neighbor in csp.get_neighbors(var) if neighbor not in assignment]
                    consistent = ac3(arcs, assignment)

                    if consistent:
                        result = backtrack(assignment, depth + 1)
                        if result is not None:
                            return result           
                 
                csp.domains = preserved_domains
                print(f"{spaces}  {var} = {val} failed, backtracking")
                csp.unassign(var, assignment)
        return None
    
    def revise(x: str, y: str, assignment: dict[str, str]) -> bool:
      is_revised = False
      for val in list(csp.domains[x]):
          # is there at least one compatible value in y's domain?
          is_compatible = False
          for val2 in csp.domains[y]:
              # here we assign temporarily to check consistency
              csp.assign(x, val, assignment)
              if csp.is_consistent(y, val2, assignment):
                  is_compatible = True
                  csp.unassign(x, assignment)
                  break
              csp.unassign(x, assignment)
          if not is_compatible:
              csp.domains[x].remove(val)
              is_revised = True
      return is_revised

    def ac3(arcs: list[tuple[str, str]], assignment: dict[str, str]) -> bool:
        queue = list(arcs)
        while queue:
            x, y = queue.pop(0)
            if revise(x, y, assignment):
                if len(csp.domains[x]) == 0:
                    
                    return False
                for neighbor in csp.get_neighbors(x):
                    if neighbor != y and neighbor not in assignment:
                        queue.append((neighbor, y))

        return True
    
    return backtrack({})


def backtracking_mrv_lcv(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking with Forward Checking + MRV + LCV.

    Tips:
    - Combine the techniques from backtracking_fc, mrv_heuristic, and lcv_heuristic.
    - MRV (Minimum Remaining Values): Select the unassigned variable with the fewest legal values.
      Tie-break by degree: prefer the variable with the most unassigned neighbors.
    - LCV (Least Constraining Value): When ordering values for a variable, prefer
      values that rule out the fewest choices for neighboring variables.
    - Use csp.get_num_conflicts(var, value, assignment) to count how many values would be ruled out for neighbors if var=value is assigned.
    """
    def backtrack(assignment: dict[str, str], depth: int = 0) -> dict[str, str] | None:
        spaces = "  " * depth
        print(f"{spaces}backtrack(depth={depth}, assigned={list(assignment.items())})")

        if csp.is_complete(assignment):
            print(f"Assignments completed: {len(assignment)}")
            return assignment

        unassigned_vars = csp.get_unassigned_variables(assignment)

        # use mrv to select next variable instead of default order
        var = mrv(unassigned_vars, assignment)  

        # use lcv instead of default order
        for val in lcv(var, assignment):  
            print(f"{spaces}Trying out {val}")

            # check if value is consistent and assign thatt
            if csp.is_consistent(var, val, assignment):
                print(f"{spaces}  Looking at {var} = {val}")
                csp.assign(var, val, assignment)

                # preserve domains before forward checking
                preserved_domains = {}
                for v in csp.domains:
                    preserved_domains[v] = list(csp.domains[v])

                # forward checking logic (same as above)
                is_empty_domain = False
                for neighbor in csp.get_neighbors(var):
                    if neighbor not in assignment:
                        for neighbor_val in list(csp.domains[neighbor]):
                            if not csp.is_consistent(neighbor, neighbor_val, assignment):
                                csp.domains[neighbor].remove(neighbor_val)
                        if len(csp.domains[neighbor]) == 0:
                            is_empty_domain = True
                            break

                if not is_empty_domain:
                    result = backtrack(assignment, depth + 1)
                    if result is not None:
                        return result

                csp.domains = preserved_domains
                print(f"{spaces}  {var} = {val} failed, backtracking")
                csp.unassign(var, assignment)
        return None
    
    def mrv(unassigned_vars: list[str], assignment: dict[str, str]) -> str:
        # picks the var with the fewest remaining values in its domain
        # tie-break: most unassigned neighbors 
        best_var = unassigned_vars[0]
        best_size = len(csp.domains[best_var])
        best_degree = 0

        # count unassigned neighbors
        for neighbor in csp.get_neighbors(best_var):
            if neighbor not in assignment:
                best_degree += 1
    
        # loop thru unassigned variables 
        for v in unassigned_vars:
            size = len(csp.domains[v])
            degree = 0
            for neighbor in csp.get_neighbors(v):
                if neighbor not in assignment:
                    degree += 1
            
            # find the one with smallest domain 
            if size < best_size or (size == best_size and degree > best_degree):
                best_var = v
                best_size = size
                best_degree = degree
        return best_var

    def lcv(var: str, assignment: dict[str, str]) -> list[str]:
        # sort domain values by number of conflicts (ascending)
        # also, least constraining value first
        values_with_conflicts = []
        for val in csp.domains[var]:
            conflicts = csp.get_num_conflicts(var, val, assignment)
            values_with_conflicts.append((conflicts, val))
        values_with_conflicts.sort()
        sorted_values = []
        for conflicts, val in values_with_conflicts:
            sorted_values.append(val)
        return sorted_values

    return backtrack({})
