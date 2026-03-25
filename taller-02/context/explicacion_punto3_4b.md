# Punto 3 y 4b — Explicacion Completa

## El Escenario

```mermaid
graph LR
    subgraph Selva Amazonica
        D["Dron - Agente MAX"]
        E1["Entrega 1"]
        E2["Entrega 2"]
        C1["Cazador 1"]
        C2["Cazador 2"]
    end

    D -->|quiere llegar a| E1
    D -->|quiere llegar a| E2
    C1 -->|quiere atrapar a| D
    C2 -->|quiere atrapar a| D

    style D fill:#1d4ed8,color:white
    style E1 fill:#15803d,color:white
    style E2 fill:#15803d,color:white
    style C1 fill:#dc2626,color:white
    style C2 fill:#dc2626,color:white
```

> El dron debe entregar suministros medicos en todos los puntos de entrega **sin ser atrapado** por los cazadores furtivos.

---

## El Juego por Turnos

```mermaid
sequenceDiagram
    participant D as Dron (MAX)
    participant C1 as Cazador 1
    participant C2 as Cazador 2

    Note over D,C2: TURNO 1 (depth = 1)
    D->>D: Elige accion
    C1->>C1: Elige accion
    C2->>C2: Elige accion

    Note over D,C2: TURNO 2 (depth = 2)
    D->>D: Elige accion
    C1->>C1: Elige accion
    C2->>C2: Elige accion

    Note over D,C2: Se repite hasta depth maximo o estado terminal
```

**Estado terminal:**
- **Victoria (+1000):** El dron visito todos los puntos de entrega.
- **Derrota (-1000):** Un cazador comparte casilla con el dron.

---

## Minimax (Punto 2 — contexto)

En Minimax los cazadores son agentes **MIN** que **siempre** eligen la peor accion para el dron.

```mermaid
graph TD
    A["MAX - Dron\nmax 7, 3 = 7"] --> B["MIN - Cazador\nmin 7, 8 = 7"]
    A --> C["MIN - Cazador\nmin 3, 9 = 3"]

    B --> D["Hoja: 7"]
    B --> E["Hoja: 8"]
    C --> F["Hoja: 3"]
    C --> G["Hoja: 9"]

    style A fill:#4CAF50,color:white
    style B fill:#f44336,color:white
    style C fill:#f44336,color:white
    style D fill:#1e3a5f,color:#93c5fd
    style E fill:#1e3a5f,color:#93c5fd
    style F fill:#1e3a5f,color:#93c5fd
    style G fill:#1e3a5f,color:#93c5fd
```

**Problema:** Minimax asume que el cazador **siempre** juega perfecto. Pero en la realidad a veces el cazador se mueve al azar, se equivoca o no ve al dron.

---

## Punto 3: Expectimax — lo que se debe implementar

En Expectimax los cazadores son nodos de **AZAR (CHANCE)** que combinan:
- Con probabilidad **(1-p):** actuan greedy (como MIN, peor caso)
- Con probabilidad **p:** actuan al azar (promedio uniforme)

```mermaid
graph TD
    A["MAX - Dron\nelige el mayor"] --> B["CHANCE - Cazador\n1-p x min + p x mean"]
    A --> C["CHANCE - Cazador\n1-p x min + p x mean"]

    B --> D["Hoja: 7"]
    B --> E["Hoja: 8"]
    C --> F["Hoja: 3"]
    C --> G["Hoja: 9"]

    style A fill:#4CAF50,color:white
    style B fill:#FF9800,color:white
    style C fill:#FF9800,color:white
    style D fill:#1e3a5f,color:#93c5fd
    style E fill:#1e3a5f,color:#93c5fd
    style F fill:#1e3a5f,color:#93c5fd
    style G fill:#1e3a5f,color:#93c5fd
```

### Formula del Nodo CHANCE

```
valor = (1 - p) * min(valores_hijos) + p * promedio(valores_hijos)
```

**Ejemplo con p = 0.3 y valores hijos [7, 8]:**

```
valor = (1 - 0.3) * min(7, 8) + 0.3 * mean(7, 8)
      = 0.7 * 7 + 0.3 * 7.5
      = 4.9 + 2.25
      = 7.15
```

| Algoritmo  | Resultado | Interpretacion                    |
|------------|-----------|-----------------------------------|
| Minimax    | 7.00      | Pesimista: asume peor caso        |
| Expectimax | 7.15      | Realista: pondera la aleatoriedad |

---

## Comparacion: Minimax vs Expectimax

```mermaid
graph LR
    subgraph Minimax
        M1["Cazador SIEMPRE\nelige lo peor\npara el dron"]
        M2["Dron muy conservador\nEvita TODO riesgo"]
        M1 --> M2
    end

    subgraph Expectimax
        E1["Cazador a veces\nse equivoca\ncon probabilidad p"]
        E2["Dron mas atrevido\nAprovecha errores"]
        E1 --> E2
    end

    style M1 fill:#f44336,color:white
    style M2 fill:#991b1b,color:white
    style E1 fill:#FF9800,color:white
    style E2 fill:#92400e,color:white
```

### Valores de p y su significado

```mermaid
graph LR
    P0["p = 0.0\nCazador 100% greedy\n= Minimax puro"] --> P03["p = 0.3\n70% greedy\n30% aleatorio"]
    P03 --> P05["p = 0.5\n50% greedy\n50% aleatorio"]
    P05 --> P1["p = 1.0\n100% aleatorio\n= Expectimax puro"]

    style P0 fill:#f44336,color:white
    style P03 fill:#FF9800,color:white
    style P05 fill:#FFC107,color:black
    style P1 fill:#4CAF50,color:white
```

| Valor de p | Comportamiento del cazador | La formula se reduce a         |
|------------|----------------------------|--------------------------------|
| p = 0      | 100% greedy                | `min(hijos)` = Minimax         |
| 0 < p < 1  | Mixto (realista)           | Combinacion ponderada          |
| p = 1      | 100% aleatorio             | `mean(hijos)` = Expectimax puro|

---

## Estructura del Arbol de Juego (2 cazadores, depth=1)

```mermaid
graph TD
    ROOT["MAX - Dron\ndepth=1"]

    ROOT -->|arriba| U["CHANCE - Cazador 1"]
    ROOT -->|abajo| D2["CHANCE - Cazador 1"]
    ROOT -->|izq| L["CHANCE - Cazador 1"]
    ROOT -->|der| R["CHANCE - Cazador 1"]

    U -->|arriba| U1["CHANCE - Cazador 2"]
    U -->|abajo| U2["CHANCE - Cazador 2"]
    U -->|izq| U3["CHANCE - Cazador 2"]

    U1 -->|arriba| L1["eval state"]
    U1 -->|abajo| L2["eval state"]
    U1 -->|izq| L3["eval state"]

    style ROOT fill:#4CAF50,color:white
    style U fill:#FF9800,color:white
    style D2 fill:#FF9800,color:white
    style L fill:#FF9800,color:white
    style R fill:#FF9800,color:white
    style U1 fill:#2196F3,color:white
    style U2 fill:#2196F3,color:white
    style U3 fill:#2196F3,color:white
    style L1 fill:#1e3a5f,color:#93c5fd
    style L2 fill:#1e3a5f,color:#93c5fd
    style L3 fill:#1e3a5f,color:#93c5fd
```

> Despues de que **todos** los agentes se mueven (dron + todos los cazadores) se completa **1 nivel de profundidad**.

---

## Funcion de Evaluacion (evaluation.py)

Cuando el arbol llega a su profundidad maxima sin un estado terminal se llama `evaluation_function(state)` para estimar que tan bueno es ese estado. Se deben considerar factores como:

```mermaid
graph TD
    EF["evaluation_function state"] --> F1["Distancia al\npunto de entrega\nmas cercano"]
    EF --> F2["Distancia a\nlos cazadores"]
    EF --> F3["Entregas\npendientes"]
    EF --> F4["Score\nactual del juego"]
    EF --> F5["Entrega alcanzable\nantes que cazador"]

    F1 -->|"menor = mejor"| SCORE["Puntaje final"]
    F2 -->|"mayor = mejor"| SCORE
    F3 -->|"menos = mejor"| SCORE
    F4 -->|"mayor = mejor"| SCORE
    F5 -->|"bonus si es segura"| SCORE

    style EF fill:#7c3aed,color:white
    style SCORE fill:#1e3a5f,color:#93c5fd
    style F1 fill:#15803d,color:white
    style F2 fill:#dc2626,color:white
    style F3 fill:#0369a1,color:white
    style F4 fill:#a16207,color:white
    style F5 fill:#4d7c0f,color:white
```

**Herramientas disponibles en `algorithms/utils.py`:**
- `bfs_distance(layout, start, goal, hunter_restricted)` — distancia BFS en pasos. Con `hunter_restricted=True` solo considera terreno libre (`.`), modelando el movimiento de cazadores.
- `dijkstra(layout, start, goal)` — costo de ruta ponderado por terreno (el dron paga 1 en libre, 2 en niebla, 3 en montana, 5 en tormenta).

**Datos del estado accesibles:**
- `state.get_drone_position()` — posicion (x, y) del dron
- `state.get_hunter_positions()` — lista de posiciones de cazadores
- `state.get_pending_deliveries()` — conjunto de entregas pendientes
- `state.get_score()` — score acumulado del juego
- `state.get_layout()` — layout del mapa
- `state.is_win()` / `state.is_lose()` — estados terminales

---

## Punto 4b: Analisis de Busqueda Adversaria

Preguntas que se deben responder en el documento PDF:

```mermaid
graph TD
    A["4b: Analisis de\nBusqueda Adversaria"] --> B["Complejidad\nMinimax"]
    A --> C["Por que Minimax\nes pesimista"]
    A --> D["Justificar funcion\nde evaluacion"]
    A --> E["Comparar tasas\nde victoria"]
    A --> F["Poda Alfa-Beta"]

    B --> B1["Tiempo: O b^d\nEspacio: O b*d"]
    C --> C1["Asume cazadores perfectos\nSobreestima el peligro\nExpectimax lo soluciona"]
    D --> D1["Que factores\nQue pesos\nPor que"]
    E --> E1["Minimax vs Expectimax\nDistintos layouts\nDistintos valores de p"]
    F --> F1["Mejor caso: O b^d/2\nPeor caso: O b^d"]

    style A fill:#7c3aed,color:white
    style B fill:#1e3a5f,color:#93c5fd
    style C fill:#1e3a5f,color:#93c5fd
    style D fill:#1e3a5f,color:#93c5fd
    style E fill:#1e3a5f,color:#93c5fd
    style F fill:#1e3a5f,color:#93c5fd
```

### 1. Complejidad de Minimax

| Metrica  | Complejidad | Descripcion                                      |
|----------|-------------|--------------------------------------------------|
| Tiempo   | O(b^d)      | b = factor de ramificacion, d = profundidad       |
| Espacio  | O(b * d)    | Solo almacena el camino actual (DFS)              |

Con **k cazadores**, cada turno completo tiene (1+k) capas de agentes, por lo que el arbol real es `O(b^(d*(1+k)))`.

### 2. Por que Minimax es pesimista con cazadores no optimos

Minimax asume que **cada cazador siempre elige la accion optima** (la peor para el dron). Si el cazador tiene 4 movimientos con valores `[-50, 30, 10, 20]`:

| Algoritmo         | Calculo                                     | Resultado |
|--------------------|---------------------------------------------|-----------|
| Minimax            | `min(-50, 30, 10, 20)`                      | -50       |
| Expectimax (p=0.5) | `0.5 * min(-50,30,10,20) + 0.5 * mean(...)` | -23.75    |

Minimax dice **-50** (catastrofe segura). Expectimax dice **-23.75** (riesgo moderado). Minimax sobreestima el peligro y hace que el dron tome decisiones demasiado conservadoras, perdiendo oportunidades de completar entregas.

### 3. Funcion de evaluacion

Describir y justificar la funcion disenada: que factores del estado se consideran, que pesos se asignan y por que. Debe balancear progreso en entregas con evasion de cazadores.

### 4. Poda Alfa-Beta

| Caso       | Nodos explorados | Descripcion                              |
|------------|------------------|------------------------------------------|
| Mejor caso | O(b^(d/2))       | Nodos ordenados perfectamente            |
| Peor caso  | O(b^d)           | Sin poda, igual que Minimax sin optimizar |

En el mejor caso Alfa-Beta explora la **raiz cuadrada** de los nodos que Minimax necesita.

---

## Archivos por modificar

```mermaid
graph LR
    subgraph Por implementar
        ADV["algorithms/adversarial.py\nExpectimaxAgent"]
        EVAL["algorithms/evaluation.py\nevaluation_function"]
    end

    subgraph Solo consultar
        GS["world/game_state.py"]
        RULES["world/rules.py"]
        UTILS["algorithms/utils.py"]
    end

    ADV -->|usa| EVAL
    ADV -->|usa| GS
    EVAL -->|usa| UTILS

    style ADV fill:#b45309,color:white
    style EVAL fill:#b45309,color:white
    style GS fill:#334155,color:#94a3b8
    style RULES fill:#334155,color:#94a3b8
    style UTILS fill:#334155,color:#94a3b8
```
