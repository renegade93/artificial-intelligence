"""
model_checking.py

Este modulo contiene las funciones de model checking proposicional.

Hint: Usa las funciones get_atoms() y evaluate() de logic_core.py.
"""

from __future__ import annotations


from src.logic_core import And, Formula, Not


def get_all_models(atoms: set[str]) -> list[dict[str, bool]]:
    """
    Genera todos los modelos posibles (asignaciones de verdad).
    Para n atomos, genera 2^n modelos.

    Args:
        atoms: Conjunto de nombres de atomos proposicionales.

    Returns:
        Lista de diccionarios, cada uno mapeando atomos a valores booleanos.

    Ejemplo:
        >>> get_all_models({'p', 'q'})
        [{'p': True, 'q': True}, {'p': True, 'q': False},
         {'p': False, 'q': True}, {'p': False, 'q': False}]

    Hint: Piensa en como representar los numeros del 0 al 2^n - 1 en binario.
          Cada bit corresponde al valor de verdad de un atomo.
    """
    
    # fill out column by column
    if not atoms:
            return [{}]
            
    atoms_list = sorted(list(atoms))
    total_rows = 2 ** len(atoms_list)
    
    # init models array
    models = []
    for _ in range(total_rows):
        models.append({})
        
    # since we are operating column-wise, the first block size is exactly half of the total rows
    block_size = total_rows // 2
    
    # iterate over atoms
    for atom in atoms_list:
        
        # we will start by adding false first
        current_value = False 
        counter = 0
        
        for row in range(total_rows):
            models[row][atom] = current_value
            counter += 1
            
            # if we hit the block size, toggle the boolean value
            if counter == block_size:
                current_value = not current_value
                counter = 0
                
        # for the next iteration, cut the block size in half for the next column 
        block_size = block_size // 2
        
    return models
    

def check_satisfiable(formula: Formula) -> tuple[bool, dict[str, bool] | None]:
    """
    Determina si una formula es satisfacible.

    Args:
        formula: Formula logica a verificar.

    Returns:
        (True, modelo) si encuentra un modelo que la satisface.
        (False, None) si es insatisfacible.

    Ejemplo:
        >>> check_satisfiable(And(Atom('p'), Not(Atom('p'))))
        (False, None)

    Hint: Genera todos los modelos con get_all_models(), luego evalua
          la formula en cada uno usando evaluate().
    """
    if not formula:
        return (False, None)
    
    # grab atoms from formula
    atoms = formula.get_atoms()

    # generate the models
    models = get_all_models(atoms)

    # evaluate each model
    for model in models:
        if formula.evaluate(model):
            return (True, model)
    return (False, None)


def check_valid(formula: Formula) -> bool:
    """
    Determina si una formula es una tautologia (valida en todo modelo).

    Args:
        formula: Formula logica a verificar.

    Returns:
        True si la formula es verdadera en todos los modelos posibles.

    Ejemplo:
        >>> check_valid(Or(Atom('p'), Not(Atom('p'))))
        True

    Hint: Una formula es valida si y solo si su negacion es insatisfacible.
          Alternativamente, verifica que sea verdadera en TODOS los modelos.
    """
    no_sat, _ = check_satisfiable(Not(formula))
    return not no_sat


def check_entailment(kb: list[Formula], query: Formula) -> bool:
    """
    Determina si KB |= query (la base de conocimiento implica la consulta).

    Args:
        kb: Lista de formulas que forman la base de conocimiento.
        query: Formula que queremos verificar si se sigue de la KB.

    Returns:
        True si la query es verdadera en todos los modelos donde la KB es verdadera.

    Ejemplo:
        >>> kb = [Implies(Atom('p'), Atom('q')), Atom('p')]
        >>> check_entailment(kb, Atom('q'))
        True

    Hint: KB |= q  si y solo si  KB ^ ~q es insatisfacible.
          Es decir, no existe un modelo donde toda la KB sea verdadera
          y la query sea falsa.
    """
    if not kb:
        return check_valid(query)
    
    kb_formula = kb[0]

    rest = kb[1:]
    for formula in rest:
        kb_formula = And(kb_formula, formula)
    
    full_formula = And(kb_formula, Not(query))
    is_sat, _ = check_satisfiable(full_formula)

    return not is_sat

def truth_table(formula: Formula) -> list[tuple[dict[str, bool], bool]]:
    """
    Genera la tabla de verdad completa de una formula.

    Args:
        formula: Formula logica.

    Returns:
        Lista de tuplas (modelo, resultado) para cada modelo posible.

    Ejemplo:
        >>> truth_table(And(Atom('p'), Atom('q')))
        [({'p': True, 'q': True}, True),
         ({'p': True, 'q': False}, False),
         ({'p': False, 'q': True}, False),
         ({'p': False, 'q': False}, False)]

    Hint: Combina get_all_models() y evaluate().
    """
    # init
    atoms = formula.get_atoms()
    models = get_all_models(atoms)

    # truth table construction
    tt = []
    for model in models:
        result = formula.evaluate(model)

        row = (model, result)
        tt.append(row)

    return tt
