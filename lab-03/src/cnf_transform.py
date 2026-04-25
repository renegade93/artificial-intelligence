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
    Iff(a, b) -> And(Implies(a, b), Implies(b, a))
    """
    if isinstance(formula, Iff):
        a = eliminate_iff(formula.left)
        b = eliminate_iff(formula.right)
        return And(Implies(a, b), Implies(b, a))
    elif isinstance(formula, Implies):
        return Implies(eliminate_iff(formula.antecedent), eliminate_iff(formula.consequent))
    elif isinstance(formula, And):
        return And(*[eliminate_iff(arg) for arg in formula.conjuncts])
    elif isinstance(formula, Or):
        return Or(*[eliminate_iff(arg) for arg in formula.disjuncts])
    elif isinstance(formula, Not):
        return Not(eliminate_iff(formula.operand))
    
    return formula


def eliminate_implication(formula: Formula) -> Formula:
    """
    Elimina implicaciones recursivamente.
    Implies(a, b) -> Or(Not(a), b)
    """
    if isinstance(formula, Implies):
        a = eliminate_implication(formula.antecedent)
        b = eliminate_implication(formula.consequent)
        return Or(Not(a), b)
    elif isinstance(formula, Iff):
        return Iff(eliminate_implication(formula.left), eliminate_implication(formula.right))
    elif isinstance(formula, And):
        return And(*[eliminate_implication(arg) for arg in formula.conjuncts])
    elif isinstance(formula, Or):
        return Or(*[eliminate_implication(arg) for arg in formula.disjuncts])
    elif isinstance(formula, Not):
        return Not(eliminate_implication(formula.operand))
        
    return formula


def push_negation_inward(formula: Formula) -> Formula:
    """
    Aplica las leyes de De Morgan y mueve negaciones hacia los atomos.
    """
    if isinstance(formula, Not):
        inner = formula.operand
        if isinstance(inner, And):
            return Or(*[push_negation_inward(Not(arg)) for arg in inner.conjuncts])
        elif isinstance(inner, Or):
            return And(*[push_negation_inward(Not(arg)) for arg in inner.disjuncts])
        elif isinstance(inner, Atom):
            return formula
        else:
            return Not(push_negation_inward(inner))
            
    elif isinstance(formula, And):
        return And(*[push_negation_inward(arg) for arg in formula.conjuncts])
    elif isinstance(formula, Or):
        return Or(*[push_negation_inward(arg) for arg in formula.disjuncts])
        
    return formula


def distribute_or_over_and(formula: Formula) -> Formula:
    """
    Distribuye Or sobre And para obtener CNF.
    Or(A, And(B, C)) -> And(Or(A, B), Or(A, C))
    """
    if isinstance(formula, And):
        return And(*[distribute_or_over_and(arg) for arg in formula.conjuncts])
    elif isinstance(formula, Or):
        dist_args = [distribute_or_over_and(arg) for arg in formula.disjuncts]
        
        for i, arg in enumerate(dist_args):
            if isinstance(arg, And):
                other_args = dist_args[:i] + dist_args[i+1:]
                new_and_args = []
                
                for and_elem in arg.conjuncts:
                    new_or = Or(*(other_args + [and_elem]))
                    new_and_args.append(distribute_or_over_and(new_or))
                    
                return And(*new_and_args)
                
        return Or(*dist_args)
        
    return formula


def flatten(formula: Formula) -> Formula:
    """
    Aplana conjunciones y disyunciones anidadas.
    And(And(a, b), c) -> And(a, b, c)
    """
    if isinstance(formula, And):
        new_args = []
        for arg in formula.conjuncts:
            flat_arg = flatten(arg)
            if isinstance(flat_arg, And):
                new_args.extend(flat_arg.conjuncts)
            else:
                new_args.append(flat_arg)
        if len(new_args) == 1:
            return new_args[0]
        return And(*new_args)
        
    elif isinstance(formula, Or):
        new_args = []
        for arg in formula.disjuncts:
            flat_arg = flatten(arg)
            if isinstance(flat_arg, Or):
                new_args.extend(flat_arg.disjuncts)
            else:
                new_args.append(flat_arg)
        if len(new_args) == 1:
            return new_args[0]
        return Or(*new_args)
        
    elif isinstance(formula, Not):
        return Not(flatten(formula.operand))
    elif isinstance(formula, Implies):
        return Implies(flatten(formula.antecedent), flatten(formula.consequent))
    elif isinstance(formula, Iff):
        return Iff(flatten(formula.left), flatten(formula.right))
        
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
