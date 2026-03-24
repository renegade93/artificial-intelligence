# Punto 3 y 4b — Explicación Completa

## El Escenario

```mermaid
graph LR
    subgraph Selva Amazónica
        D["🛩️ Dron<br/>Agente MAX"]
        E1["📦 Entrega 1"]
        E2["📦 Entrega 2"]
        C1["🏴‍☠️ Cazador 1"]
        C2["🏴‍☠️ Cazador 2"]
    end

    D -->|"Quiere llegar a"| E1
    D -->|"Quiere llegar a"| E2
    C1 -->|"Quiere atrapar a"| D
    C2 -->|"Quiere atrapar a"| D
```

> El dron debe entregar suministros médicos en todos los puntos de entrega **sin ser atrapado** por los cazadores furtivos.

---

## El Juego por Turnos

```mermaid
sequenceDiagram
    participant D as 🛩️ Dron (MAX)
    participant C1 as 🏴‍☠️ Cazador 1
    participant C2 as 🏴‍☠️ Cazador 2

    Note over D,C2: === TURNO 1 (depth = 1) ===
    D->>D: Elige acción (↑↓←→)
    C1->>C1: Elige acción
    C2->>C2: Elige acción

    Note over D,C2: === TURNO 2 (depth = 2) ===
    D->>D: Elige acción
    C1->>C1: Elige acción
    C2->>C2: Elige acción

    Note over D,C2: Se repite hasta depth máximo o estado terminal
```

**Estado terminal:**
- **Victoria (+1000):** El dron visitó todos los puntos de entrega
- **Derrota (-1000):** Un cazador comparte casilla con el dron

---

## Punto 2: Minimax (lo que ya hicieron tus compañeros)

En Minimax, los cazadores son agentes **MIN** que **siempre** eligen la peor acción para el dron.

```mermaid
graph TD
    A["🛩️ MAX<br/>Dron elige<br/>max(7, 3) = 7"] --> B["🏴‍☠️ MIN<br/>Cazador elige<br/>min(7, 8) = 7"]
    A --> C["🏴‍☠️ MIN<br/>Cazador elige<br/>min(3, 9) = 3"]

    B --> D["Hoja: 7"]
    B --> E["Hoja: 8"]
    C --> F["Hoja: 3"]
    C --> G["Hoja: 9"]

    style A fill:#4CAF50,color:white
    style B fill:#f44336,color:white
    style C fill:#f44336,color:white
```

**Problema:** Minimax asume que el cazador **siempre** juega perfecto. Pero en la realidad, a veces el cazador se mueve al azar (es humano, se equivoca, no ve al dron).

---

## Punto 3: Expectimax (lo que TÚ implementas)

En Expectimax, los cazadores son nodos de **AZAR (CHANCE)** que combinan:
- Con probabilidad **(1-p):** actúan greedy (como MIN, peor caso)
- Con probabilidad **p:** actúan al azar (promedio uniforme)

```mermaid
graph TD
    A["🛩️ MAX<br/>Dron elige<br/>max(6.5, 5.4) = 6.5"] --> B["🎲 CHANCE<br/>Cazador<br/>(1-p)·min + p·mean"]
    A --> C["🎲 CHANCE<br/>Cazador<br/>(1-p)·min + p·mean"]

    B --> D["Hoja: 7"]
    B --> E["Hoja: 8"]
    C --> F["Hoja: 3"]
    C --> G["Hoja: 9"]

    style A fill:#4CAF50,color:white
    style B fill:#FF9800,color:white
    style C fill:#FF9800,color:white
```

### La Fórmula del Nodo CHANCE

```
valor = (1 - p) × min(valores_hijos) + p × promedio(valores_hijos)
```

**Ejemplo con p = 0.3 y valores hijos [7, 8]:**
```
valor = (1 - 0.3) × min(7, 8) + 0.3 × mean(7, 8)
       = 0.7 × 7 + 0.3 × 7.5
       = 4.9 + 2.25
       = 7.15
```

**Comparado con Minimax:**
- Minimax diría: `min(7, 8) = 7` (pesimista)
- Expectimax dice: `7.15` (más realista)

---

## Comparación Visual: Minimax vs Expectimax

```mermaid
graph LR
    subgraph Minimax
        M1["Cazador SIEMPRE<br/>elige lo peor<br/>para el dron"]
        M2["Resultado:<br/>Dron muy conservador<br/>Evita TODO riesgo"]
        M1 --> M2
    end

    subgraph Expectimax
        E1["Cazador a veces<br/>se equivoca<br/>con probabilidad p"]
        E2["Resultado:<br/>Dron más atrevido<br/>Aprovecha errores"]
        E1 --> E2
    end
```

### Valores de p y su significado

```mermaid
graph LR
    P0["p = 0.0<br/>Cazador 100% greedy<br/>= Minimax puro"] --> P03["p = 0.3<br/>70% greedy<br/>30% aleatorio"]
    P03 --> P05["p = 0.5<br/>50% greedy<br/>50% aleatorio"]
    P05 --> P1["p = 1.0<br/>100% aleatorio<br/>= Expectimax puro"]

    style P0 fill:#f44336,color:white
    style P03 fill:#FF9800,color:white
    style P05 fill:#FFC107,color:black
    style P1 fill:#4CAF50,color:white
```

---

## Estructura del Árbol de Juego (con 2 cazadores, depth=1)

```mermaid
graph TD
    ROOT["🛩️ MAX - Dron<br/>depth=1"]

    ROOT -->|"↑"| U["🎲 CHANCE - Cazador 1"]
    ROOT -->|"↓"| D2["🎲 CHANCE - Cazador 1"]
    ROOT -->|"←"| L["🎲 CHANCE - Cazador 1"]
    ROOT -->|"→"| R["🎲 CHANCE - Cazador 1"]

    U -->|"↑"| U1["🎲 CHANCE - Cazador 2"]
    U -->|"↓"| U2["🎲 CHANCE - Cazador 2"]
    U -->|"←"| U3["🎲 CHANCE - Cazador 2"]

    U1 -->|"↑"| L1["eval()"]
    U1 -->|"↓"| L2["eval()"]
    U1 -->|"←"| L3["eval()"]

    style ROOT fill:#4CAF50,color:white
    style U fill:#FF9800,color:white
    style D2 fill:#FF9800,color:white
    style L fill:#FF9800,color:white
    style R fill:#FF9800,color:white
    style U1 fill:#2196F3,color:white
    style U2 fill:#2196F3,color:white
    style U3 fill:#2196F3,color:white
```

> Después de que **todos** los agentes se mueven (dron + todos los cazadores), se completa **1 nivel de profundidad**.

---

## La Función de Evaluación (evaluation.py)

Cuando el árbol llega a su profundidad máxima sin un estado terminal, se llama `evaluation_function(state)` para estimar qué tan bueno es ese estado.

```mermaid
graph TD
    EF["evaluation_function(state)"] --> F1["📦 Distancia al<br/>punto de entrega<br/>más cercano<br/>(menor = mejor)"]
    EF --> F2["🏴‍☠️ Distancia a<br/>los cazadores<br/>(mayor = mejor)"]
    EF --> F3["📋 Entregas<br/>pendientes<br/>(menos = mejor)"]
    EF --> F4["🏆 Score<br/>actual del juego"]

    F1 -->|"Peso alto"| SCORE["Puntaje final<br/>entre -1000 y +1000"]
    F2 -->|"Peso medio"| SCORE
    F3 -->|"Peso alto"| SCORE
    F4 -->|"Peso bajo"| SCORE
```

---

## Punto 4b: Análisis que debes escribir

```mermaid
graph TD
    A["4b: Análisis de<br/>Búsqueda Adversaria"] --> B["Complejidad<br/>Minimax"]
    A --> C["¿Por qué Minimax<br/>es pesimista?"]
    A --> D["Justificar función<br/>de evaluación"]
    A --> E["Comparar tasas<br/>de victoria"]
    A --> F["Poda Alfa-Beta"]

    B --> B1["Tiempo: O(b^d)<br/>Espacio: O(b·d)<br/>b=ramificación, d=profundidad"]
    C --> C1["Asume cazadores perfectos<br/>Sobreestima el peligro<br/>Expectimax lo soluciona"]
    D --> D1["Qué factores usas<br/>Qué pesos<br/>Por qué esos pesos"]
    E --> E1["MinimaxAgent vs ExpectimaxAgent<br/>Distintos layouts<br/>Distintos valores de p"]
    F --> F1["Mejor caso: O(b^(d/2))<br/>Peor caso: O(b^d)<br/>= sin poda"]
```

---

## Resumen: ¿Qué archivos tocas?

```mermaid
graph LR
    subgraph "Tu trabajo"
        ADV["algorithms/adversarial.py<br/>→ ExpectimaxAgent"]
        EVAL["algorithms/evaluation.py<br/>→ evaluation_function"]
        PDF["documento.pdf<br/>→ Análisis 4b"]
    end

    subgraph "Solo consultar"
        GS["world/game_state.py"]
        RULES["world/rules.py"]
        UTILS["algorithms/utils.py"]
    end

    ADV -->|"usa"| EVAL
    ADV -->|"usa"| GS
    EVAL -->|"usa"| UTILS
```
