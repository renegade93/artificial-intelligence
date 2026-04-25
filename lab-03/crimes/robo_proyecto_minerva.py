"""
robo_proyecto_minerva.py — El Robo del Proyecto Minerva (CASO BONO)

La noche antes de que el grupo de investigación ISIS-1 sometiera la patente de su
sistema de IA biomédica (Proyecto Minerva), el servidor con todos los datos fue
robado del laboratorio ML-403 en el Bloque Mario Laserna de la Universidad de
los Andes. El grupo rival de Ingeniería Eléctrica, liderado por la profesora
Daniela Ríos, perdería millones en financiación si la patente de ISIS-1 se
aprobaba primero. Su estudiante de doctorado, Tomás Pedraza, cuya tesis depende
directamente del financiamiento del grupo rival, tenía acceso legítimo al ML por
un convenio de laboratorio compartido firmado meses atrás. Las cámaras lo ubican
cerca del ML-403 a las 11 PM y no puede acreditar su paradero esa noche. Una
becaria escuchó a Daniela hablando por teléfono sobre el servidor esa misma
madrugada, horas antes de que el robo se hiciera público, y Daniela nunca
reportó nada. El técnico del edificio, Ernesto Vargas, aparece en cámara en
recepción toda la noche. La investigadora Claudia Nieto estaba en un congreso
en Medellín con registros de hotel verificados.

Como detective, he llegado a las siguientes conclusiones:
Quien tiene acceso al laboratorio y motivo económico es sospechoso del robo.
Un sospechoso visto en la escena y sin coartada verificada es culpable.
Un miembro del grupo rival que conocía el robo y guardó silencio es cómplice.
Todo culpable del robo está involucrado en el crimen.
Todo cómplice también está involucrado en el crimen.
"""

from src.crime_case import CrimeCase, QuerySpec
from src.predicate_logic import (
    ExistsGoal,
    ForallGoal,
    KnowledgeBase,
    Predicate,
    Rule,
    Term,
)


def crear_kb() -> KnowledgeBase:
    """Construye la KB según la narrativa del módulo."""
    kb = KnowledgeBase()

    # Constantes
    tomas   = Term("tomas")
    daniela = Term("daniela")
    ernesto = Term("ernesto")
    claudia = Term("claudia")

    # Hechos: personasss
    kb.add_fact(Predicate("persona", (tomas,)))
    kb.add_fact(Predicate("persona", (daniela,)))
    kb.add_fact(Predicate("persona", (ernesto,)))
    kb.add_fact(Predicate("persona", (claudia,)))

    #Hechos: grupos
    kb.add_fact(Predicate("grupo_rival", (tomas,)))
    kb.add_fact(Predicate("grupo_rival", (daniela,)))

    # Hechos: acesss
    kb.add_fact(Predicate("acceso_lab", (tomas,)))
    kb.add_fact(Predicate("acceso_lab", (ernesto,)))

    #Hechos:ubicacion
    kb.add_fact(Predicate("visto_en_escena", (tomas,)))

    # Hechos: motivo
    kb.add_fact(Predicate("motivo_economico", (tomas,)))
    kb.add_fact(Predicate("motivo_economico", (daniela,)))

    # Hechos: coartada
    kb.add_fact(Predicate("sin_coartada", (tomas,)))
    kb.add_fact(Predicate("descartado", (ernesto,)))
    kb.add_fact(Predicate("descartado", (claudia,)))

    # Hechos: conocimientos
    kb.add_fact(Predicate("conoce_robo", (daniela,)))
    kb.add_fact(Predicate("guardo_silencio", (daniela,)))

    # regla

    # Regla 1: sospechoso($X) :- acceso_lab($X), motivo_economico($X)
    kb.add_rule(
        Rule(
            head=Predicate("sospechoso", (Term("$X"),)),
            body=(
                Predicate("acceso_lab", (Term("$X"),)),
                Predicate("motivo_economico", (Term("$X"),)),
            ),
        )
    )

    # Regla 2: culpable($X) :- sospechoso($X), visto_en_escena($X), sin_coartada($X)
    kb.add_rule(
        Rule(
            head=Predicate("culpable", (Term("$X"),)),
            body=(
                Predicate("sospechoso", (Term("$X"),)),
                Predicate("visto_en_escena", (Term("$X"),)),
                Predicate("sin_coartada", (Term("$X"),)),
            ),
        )
    )

    # Regla 3: complice($X) :- grupo_rival($X), conoce_robo($X), guardo_silencio($X)
    kb.add_rule(
        Rule(
            head=Predicate("complice", (Term("$X"),)),
            body=(
                Predicate("grupo_rival", (Term("$X"),)),
                Predicate("conoce_robo", (Term("$X"),)),
                Predicate("guardo_silencio", (Term("$X"),)),
            ),
        )
    )

    # Regla 4: involucrado($X) :- culpable($X)
    kb.add_rule(
        Rule(
            head=Predicate("involucrado", (Term("$X"),)),
            body=(Predicate("culpable", (Term("$X"),)),),
        )
    )

    # Regla 5: involucrado($X) :- complice($X)
    kb.add_rule(
        Rule(
            head=Predicate("involucrado", (Term("$X"),)),
            body=(Predicate("complice", (Term("$X"),)),),
        )
    )

    return kb


CASE = CrimeCase(
    id="robo_proyecto_minerva",
    title="El Robo del Proyecto Minerva",
    suspects=("tomas", "daniela", "ernesto", "claudia"),
    narrative=__doc__,
    description=(
        "El servidor con la patente del Proyecto Minerva fue robado del laboratorio "
        "ML-403 del Bloque Mario Laserna. Dos investigadores del grupo rival tenían "
        "motivo económico. Uno tenía acceso, fue visto en la escena y no tiene coartada; "
        "la otra sabía del robo y guardó silencio. Razona sobre acceso físico, motivo, "
        "coartadas y encubrimiento."
    ),
    create_kb=crear_kb,
    queries=(
        QuerySpec(
            description="¿Ernesto está descartado?",
            goal=Predicate("descartado", (Term("ernesto"),)),
        ),
        QuerySpec(
            description="¿Claudia está descartada?",
            goal=Predicate("descartado", (Term("claudia"),)),
        ),
        QuerySpec(
            description="¿Tomás es culpable del robo?",
            goal=Predicate("culpable", (Term("tomas"),)),
        ),
        QuerySpec(
            description="¿Daniela es cómplice del robo?",
            goal=Predicate("complice", (Term("daniela"),)),
        ),
        QuerySpec(
            description="¿Tomás está involucrado en el crimen?",
            goal=Predicate("involucrado", (Term("tomas"),)),
        ),
        QuerySpec(
            description="¿Daniela está involucrada en el crimen?",
            goal=Predicate("involucrado", (Term("daniela"),)),
        ),
        QuerySpec(
            description="¿Existe al menos un culpable del robo?",
            goal=ExistsGoal("$X", Predicate("culpable", (Term("$X"),))),
        ),
        QuerySpec(
            description="¿Todos los miembros del grupo rival tenían motivo económico?",
            goal=ForallGoal(
                "$X",
                Predicate("grupo_rival", (Term("$X"),)),
                Predicate("motivo_economico", (Term("$X"),)),
            ),
        ),
    ),
)
