"""
robo_expreso_sur.py — El Robo en el Expreso del Sur

El collar de esmeraldas de la Marquesa desapareció del vagón privado del tren nocturno.
Elena fue vista en el vagón privado durante el robo; sus huellas están en el estuche de joyas.
Don Rodrigo fue grabado por la cámara de seguridad en el vagón de equipaje durante toda la noche.
El vagón de equipaje es el extremo opuesto al vagón privado; es imposible haber estado en ambos a la vez.
La Marquesa es la víctima directa del robo y presenció el incidente.
La Marquesa acusa a Elena.
Victor declara que Elena estuvo con él en el vagón comedor toda la noche.
Elena declara que Victor estuvo con ella en el vagón comedor toda la noche.

Como detective, he llegado a las siguientes conclusiones:
/Quien fue grabado en cámara en un lugar alejado de la escena durante el crimen está descartado.
/La víctima del crimen no tiene razón para mentir; es testigo imparcial.
/La acusación de un testigo imparcial es creíble.
/Quien estaba en la escena y es acusado de forma creíble es culpable.
/Quien da coartada a un culpable lo está defendiendo.
/Si dos personas se dan coartada mutuamente, tienen una alianza de coartadas entre sí.
"""

from src.crime_case import CrimeCase, QuerySpec
from src.predicate_logic import ExistsGoal, KnowledgeBase, Predicate, Rule, Term


def crear_kb() -> KnowledgeBase:
    """Construye la KB según la narrativa del módulo."""
    kb = KnowledgeBase()

    # Constantes del caso
    elena          = Term("elena")
    victor         = Term("victor")
    don_rodrigo    = Term("don_rodrigo")
    marquesa       = Term("marquesa")
    estuche_joyas  = Term("estuche_joyas")
    vagon_equipaje = Term("vagon_equipaje")
    vagon_privado = Term("vagon_privado")

    # === YOUR CODE HERE ===
    #Hechos
    kb.add_fact(Predicate("robo", (estuche_joyas,)))
    kb.add_fact(Predicate("huellas_en", (elena, estuche_joyas)))
    kb.add_fact(Predicate("visto_en", (elena, vagon_privado)))
    kb.add_fact(Predicate("lugar_alejado", (vagon_equipaje, vagon_privado)))    
    kb.add_fact(Predicate("grabado_en", (don_rodrigo, vagon_equipaje)))
    kb.add_fact(Predicate("victima", (marquesa,)))
    kb.add_fact(Predicate("acusa_a", (marquesa, elena)))
    kb.add_fact(Predicate("declara_con_coartada", (victor, elena)))
    kb.add_fact(Predicate("declara_con_coartada", (elena, victor)))
    kb.add_fact(Predicate("distinto", (elena, victor)))
    kb.add_fact(Predicate("distinto", (victor, elena)))
    
    #Reglas
    kb.add_rule(Rule(
    head=Predicate("descartado", (Term("$X"),)),
    body=(
        Predicate("grabado_en", (Term("$X"), Term("$Y"))),
        Predicate("lugar_alejado", (Term("$Y"), vagon_privado)),
    )
    ))
    
    kb.add_rule(Rule(
        head=Predicate("testigo_imparcial", (Term("$X"),)),
        body=(Predicate("victima", (Term("$X"),)),)
    ))
    
    kb.add_rule(Rule(
        head=Predicate("acusacion_creible", (Term("$X"), Term("$Y"))),
        body=(Predicate("acusa_a", (Term("$X"), Term("$Y"))), Predicate("testigo_imparcial", (Term("$X"),)))
    ))
    
    kb.add_rule(Rule(
    head=Predicate("culpable", (Term("$X"),)),
    body=(
        Predicate("visto_en", (Term("$X"), vagon_privado)),
        Predicate("acusacion_creible", (Term("$Y"), Term("$X"))),
    )
    ))
    
    kb.add_rule(Rule(
        head=Predicate("defiende_al_culpable", (Term("$X"),)),
        body=(Predicate("declara_con_coartada", (Term("$X"), Term("$Y"))), Predicate("culpable", (Term("$Y"),)))
    ))
    
    kb.add_rule(Rule(
    head=Predicate("alianza_coartadas", (Term("$X"), Term("$Y"))),
    body=(
        Predicate("declara_con_coartada", (Term("$X"), Term("$Y"))),
        Predicate("declara_con_coartada", (Term("$Y"), Term("$X"))),
        Predicate("distinto", (Term("$X"), Term("$Y"))),
    )
    ))

    # === END YOUR CODE ===

    return kb


CASE = CrimeCase(
    id="robo_expreso_sur",
    title="El Robo en el Expreso del Sur",
    suspects=("elena", "victor", "don_rodrigo"),
    narrative=__doc__,
    description=(
        "El collar de la Marquesa desapareció en un tren nocturno. "
        "Don Rodrigo tiene coartada de cámara. Elena estaba en la escena con huellas en el estuche. "
        "La víctima la acusa. Victor y Elena se cubren mutuamente."
    ),
    create_kb=crear_kb,
    queries=(
        QuerySpec(
            description="¿Don Rodrigo está descartado?",
            goal=Predicate("descartado", (Term("don_rodrigo"),)),
        ),
        QuerySpec(
            description="¿La acusación de la Marquesa contra Elena es creíble?",
            goal=Predicate("acusacion_creible", (Term("marquesa"), Term("elena"))),
        ),
        QuerySpec(
            description="¿Elena es culpable?",
            goal=Predicate("culpable", (Term("elena"),)),
        ),
        QuerySpec(
            description="¿Victor defiende al culpable?",
            goal=Predicate("defiende_al_culpable", (Term("victor"),)),
        ),
        QuerySpec(
            description="¿Existe alianza de coartadas entre Elena y Victor?",
            goal=ExistsGoal("$X", Predicate("alianza_coartadas", (Term("$X"), Term("victor")))),
        ),
    ),
)
