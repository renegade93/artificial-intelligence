"""
cnf_transform.py — Transformaciones a Forma Normal Conjuntiva (CNF).
El pipeline completo to_cnf() llama a todas las transformaciones en orden.
"""

from __future__ import annotations

from src.logic_core import And, Atom, Formula, Iff, Implies, Not, Or

import src.logic_core as logic_core

# --- FUNCION GUÍA SUMINISTRADA COMPLETA ---


def eliminate_double_negation(formula: Formula) -> Formula:
    """
    Elimina dobles negaciones recursivamente.

    Transformacion:
        Not(Not(a)) -> a

    Se aplica recursivamente hasta que no queden dobles negaciones.

    Ejemplo:
        >>> eliminate_double_negation(Not(Not(Atom('p'))))
        Atom('p')
        >>> eliminate_double_negation(Not(Not(Not(Atom('p')))))
        Not(Atom('p'))
    """
    if isinstance(formula, Atom):
        return formula
    if isinstance(formula, Not):
        if isinstance(formula.operand, Not):
            return eliminate_double_negation(formula.operand.operand)
        return Not(eliminate_double_negation(formula.operand))
    if isinstance(formula, And):
        return And(*(eliminate_double_negation(c) for c in formula.conjuncts))
    if isinstance(formula, Or):
        return Or(*(eliminate_double_negation(d) for d in formula.disjuncts))
    return formula


# --- FUNCIONES QUE DEBEN IMPLEMENTAR ---


def eliminate_iff(formula: Formula) -> Formula:
    """
    Elimina bicondicionales recursivamente.

    Transformacion:
        Iff(a, b) -> And(Implies(a, b), Implies(b, a))

    Debe aplicarse recursivamente a todas las sub-formulas.

    Ejemplo:
        >>> eliminate_iff(Iff(Atom('p'), Atom('q')))
        And(Implies(Atom('p'), Atom('q')), Implies(Atom('q'), Atom('p')))

    Hint: Usa pattern matching sobre el tipo de la formula.
          Para cada tipo, aplica eliminate_iff recursivamente a los operandos,
          y solo transforma cuando encuentras un Iff.
    """
    # === YOUR CODE HERE ===
    match formula:
        case Iff(a, b):
            return And(Implies(eliminate_iff(a), eliminate_iff(b)), 
                       Implies(eliminate_iff(b), eliminate_iff(a)))
        case Implies(a, b):
            return Implies(eliminate_iff(a), eliminate_iff(b))
        case And(*args):
            return And(*[eliminate_iff(arg) for arg in args])
        case Or(*args):
            return Or(*[eliminate_iff(arg) for arg in args])
        case Not(a):
            return Not(eliminate_iff(a))
        case _:
            return formula
    # === END YOUR CODE ===


def eliminate_implication(formula: Formula) -> Formula:
    """
    Elimina implicaciones recursivamente.

    Transformacion:
        Implies(a, b) -> Or(Not(a), b)

    Debe aplicarse recursivamente a todas las sub-formulas.

    Ejemplo:
        >>> eliminate_implication(Implies(Atom('p'), Atom('q')))
        Or(Not(Atom('p')), Atom('q'))

    Hint: Similar a eliminate_iff. Recorre recursivamente y transforma
          solo los nodos Implies.
    """
    # === YOUR CODE HERE ===
    match formula:
        case Implies(a, b):
            return Or(Not(eliminate_implication(a)), eliminate_implication(b))
        case Iff(a, b):
            return Iff(eliminate_implication(a), eliminate_implication(b))
        case And(*args):
            return And(*[eliminate_implication(arg) for arg in args])
        case Or(*args):
            return Or(*[eliminate_implication(arg) for arg in args])
        case Not(a):
            return Not(eliminate_implication(a))
        case _:
            return formula
    # === END YOUR CODE ===


def push_negation_inward(formula: Formula) -> Formula:
    """
    Aplica las leyes de De Morgan y mueve negaciones hacia los atomos.

    Transformaciones:
        Not(And(a, b, ...)) -> Or(Not(a), Not(b), ...)   (De Morgan)
        Not(Or(a, b, ...))  -> And(Not(a), Not(b), ...)   (De Morgan)

    Debe aplicarse recursivamente a todas las sub-formulas.

    Ejemplo:
        >>> push_negation_inward(Not(And(Atom('p'), Atom('q'))))
        Or(Not(Atom('p')), Not(Atom('q')))
        >>> push_negation_inward(Not(Or(Atom('p'), Atom('q'))))
        And(Not(Atom('p')), Not(Atom('q')))

    Hint: Cuando encuentres un Not, revisa que hay adentro:
          - Si es Not(And(...)): aplica De Morgan para convertir en Or de negaciones.
          - Si es Not(Or(...)): aplica De Morgan para convertir en And de negaciones.
          - Si es Not(Atom): dejar como esta.
          Para And y Or sin negacion encima, simplemente recursa sobre los hijos.

    Nota: Esta funcion se llama DESPUES de eliminar Iff e Implies,
          asi que no necesitas manejar esos tipos.
    """
    # === YOUR CODE HERE ===
    match formula:
        case Not(And(*args)):
            return Or(*[push_negation_inward(Not(arg)) for arg in args])
        case Not(Or(*args)):
            return And(*[push_negation_inward(Not(arg)) for arg in args])
        case Not(Atom(name)):
            return formula
        case Not(a):
            # Recurso de seguridad por si hay otras estructuras (ej: doble negación)
            return Not(push_negation_inward(a))
        case And(*args):
            return And(*[push_negation_inward(arg) for arg in args])
        case Or(*args):
            return Or(*[push_negation_inward(arg) for arg in args])
        case _:
            return formula
    # === END YOUR CODE ===


def distribute_or_over_and(formula: Formula) -> Formula:
    """
    Distribuye Or sobre And para obtener CNF.

    Transformacion:
        Or(A, And(B, C)) -> And(Or(A, B), Or(A, C))

    Debe aplicarse recursivamente hasta que no queden Or que contengan And.

    Ejemplo:
        >>> distribute_or_over_and(Or(Atom('p'), And(Atom('q'), Atom('r'))))
        And(Or(Atom('p'), Atom('q')), Or(Atom('p'), Atom('r')))

    Hint: Para un nodo Or, primero distribuye recursivamente en los hijos.
          Luego busca si algun hijo es un And. Si lo encuentras, aplica la
          distribucion y recursa sobre el resultado (podria haber mas).
          Para And, simplemente recursa sobre cada conjuncion.
          Atomos y Not se retornan sin cambio.

    Nota: Esta funcion se llama DESPUES de mover negaciones hacia adentro,
          asi que solo veras Atom, Not(Atom), And y Or.
    """
    # === YOUR CODE HERE ===
    match formula:
        case And(*args):
            return And(*[distribute_or_over_and(arg) for arg in args])
        case Or(*args):
            # Primero distribuimos recursivamente en los hijos
            dist_args = [distribute_or_over_and(arg) for arg in args]
            
            # Buscamos si algún hijo es un And
            for i, arg in enumerate(dist_args):
                match arg:
                    case And(*and_args):
                        # Separamos el resto de los argumentos del Or original
                        other_args = dist_args[:i] + dist_args[i+1:]
                        new_and_args = []
                        
                        # Distribuimos el resto de los elementos del Or sobre cada elemento del And
                        for and_elem in and_args:
                            new_or = Or(*(other_args + [and_elem]))
                            # Recursamos en caso de que este nuevo Or contenga más Ands anidados
                            new_and_args.append(distribute_or_over_and(new_or))
                            
                        return And(*new_and_args)
            
            # Si no se encontró ningún And, devolvemos el Or procesado
            return Or(*dist_args)
        case _:
            return formula
    # === END YOUR CODE ===


def flatten(formula: Formula) -> Formula:
    """
    Aplana conjunciones y disyunciones anidadas.

    Transformaciones:
        And(And(a, b), c) -> And(a, b, c)
        Or(Or(a, b), c)   -> Or(a, b, c)

    Debe aplicarse recursivamente.

    Ejemplo:
        >>> flatten(And(And(Atom('a'), Atom('b')), Atom('c')))
        And(Atom('a'), Atom('b'), Atom('c'))
        >>> flatten(Or(Or(Atom('a'), Atom('b')), Atom('c')))
        Or(Atom('a'), Atom('b'), Atom('c'))

    Hint: Para un And, recorre cada hijo. Si un hijo tambien es And,
          agrega sus conjuncts directamente en vez de agregar el And.
          Igual para Or con sus disjuncts.
          Si al final solo queda 1 elemento, retornalo directamente.
    """
    # === YOUR CODE HERE ===
    match formula:
        case And(*args):
            new_args = []
            for arg in args:
                flat_arg = flatten(arg)
                match flat_arg:
                    case And(*sub_args):
                        new_args.extend(sub_args)
                    case _:
                        new_args.append(flat_arg)
            if len(new_args) == 1:
                return new_args[0]
            return And(*new_args)
            
        case Or(*args):
            new_args = []
            for arg in args:
                flat_arg = flatten(arg)
                match flat_arg:
                    case Or(*sub_args):
                        new_args.extend(sub_args)
                    case _:
                        new_args.append(flat_arg)
            if len(new_args) == 1:
                return new_args[0]
            return Or(*new_args)
            
        case Not(a):
            return Not(flatten(a))
        case Implies(a, b):
            return Implies(flatten(a), flatten(b))
        case Iff(a, b):
            return Iff(flatten(a), flatten(b))
        case _:
            return formula
    # === END YOUR CODE ===


# --- PIPELINE COMPLETO ---


def to_cnf(formula: Formula) -> Formula:
    """
    [DADO] Pipeline completo de conversion a CNF.

    Aplica todas las transformaciones en el orden correcto:
    1. Eliminar bicondicionales (Iff)
    2. Eliminar implicaciones (Implies)
    3. Mover negaciones hacia adentro (Not)
    4. Eliminar dobles negaciones (Not Not)
    5. Distribuir Or sobre And
    6. Aplanar conjunciones/disyunciones

    Ejemplo:
        >>> to_cnf(Implies(Atom('p'), And(Atom('q'), Atom('r'))))
        And(Or(Not(Atom('p')), Atom('q')), Or(Not(Atom('p')), Atom('r')))
    """
    formula = eliminate_iff(formula)
    formula = eliminate_implication(formula)
    formula = push_negation_inward(formula)
    formula = eliminate_double_negation(formula)
    formula = distribute_or_over_and(formula)
    formula = flatten(formula)
    return formula
