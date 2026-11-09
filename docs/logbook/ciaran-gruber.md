# Ciaran Gruber: interfaces, observation model and simulation framework

[Back to the team logbook](README.md)

| | |
| --- | --- |
| Role | Interfaces between policy components; observation and configuration data model; logged-data simulation and visualisation |
| Figures | `ciaran/figures/` |

---

## 1. Where my parts fit (Standard 1: Overall subsystem)

![Overall block diagram](ciaran/figures/Overall%20block%20diagram.png)

*Figure: Overall block diagram of the robotic system - my contributions laid on the interface between the dream input/output and the policy nodes themselves.*

---

## 2. How it works (Standard 2: Theory / Algorithms)

### Logging

The diagram below shows the process of logging while the car is performing various steps

```mermaid
flowchart TD
    A[Dream: Car begins step] --> A2[Convert Dream observations into serialisable dataclass]
    A2 --> B{Logging enabled?}
    B -->|Yes| C[Serialise observations and append to observations log]
    B -->|No| D[Run policy and compute actions]
    C --> D
    D --> E{Logging enabled?}
    E -->|Yes| F[Serialise actions and append to observations log]
    E -->|No| G2[Deserialise actions data into readable car actions]
    F --> G2
    G2 --> G[Dream: Send car actions to car]

    classDef logging fill:#1d4ed8,stroke:#1e3a8a,stroke-width:2px,color:#fff
    classDef convert fill:#15803d,stroke:#14532d,stroke-width:2px,color:#fff
    class B,C,E,F logging
    class A2,G2 convert
```

### Concurrent policy actions

Attempted design for concurrent camera panning with separate algorithm tracks.

![Modified concurrent camera design](ciaran/figures/Modified%20concurrent%20camera%20design.png)

*Figure: Modified block diagram designed for concurrent execution of policy nodes.*

---

## 3. Use of single-step simulation to aid in architectural design (Standard 3)

### Design objective

To modify the architecture of the project to improve extensibility in future development without affecting car output in a meaningful manner.

### How simulation was used

```mermaid
flowchart TD
    A[Collect suitable scenario data] --> B[Develop modifications e.g. adding CarMemory]
    B --> C[Compare each simulated step with the expected results]
    C --> D{All actions match within 1e-6?}
    D -->|No| B
    D -->|Yes| E[Accept the modification]
```

Car actions were allowed to differ by at most $1 \times 10^{-6}$. That keeps the commanded output consistent with the baseline while ignoring floating-point noise that does not change behaviour. This provided for quick development without requiring testing with a controlled and consistent environment.

---

## 4. Solving an issue with the car veering into cones (Standard 4: Real-world experimentation)

### Engineering question

Why, on curves, was the car veering into cones?

Simulation and unit tests showed the car successfully navigating the course, so the failure only showed up on the real track.

![Track photo for fail_curvy_road](ciaran/figures/fail_curvy_road.jpg)

*Figure: The track that the car failed on (note: the obstacle was moved aside for the experimentation).*

### Identifying core influential variables

Influential variables included:

- **Curvature of the course** — courses with no curvature showed no issues.
- **Size of gaps in the cones** — the car expected a certain gap between cones.
- **Presence of obstacles** — in some circumstances an obstacle could obscure cones (to minimise bias, that obstacle was removed for the investigation).

### Running the experiment

```mermaid
flowchart TD
    A[Set up the track] --> B[Run the car with data logging enabled]
    B --> C[Convert logged data into an appropriate scenario]
    C --> D[View with the scenario renderer and develop a hypothesis]
    D --> E[Rerun the scenario and debug the code]
    E --> F[Inspect centre-line overlays e.g. 26 Oct frame 60]
```

`fail_curvy_road` replayed with the 26 Oct 2026 policy (`d1d8096`) and that day's `ai4r_policy.yaml`. The scenario renderer is unchanged aside from overlaying the lane edges and centre line that `MovementPolicy` produced after the cone filter and local map.

![fail_curvy_road centre line under 26 Oct policy](ciaran/figures/fail_curvy_road_centreline_oct26.gif)

*Figure: Real-time* `fail_curvy_road` *scenario replay with the centre-line overlay.*

![fail_curvy_road frame 60 centre line under 26 Oct policy](ciaran/figures/fail_curvy_road_frame60_centreline_oct26.png)

*Figure: frame 60 of the failing track scenario, where the detector reports* `NO_LANE`*.*

### Resolution

The car was unable to identify a proper centre line because blue cones far in the distance conflicted with the yellow cones nearby.

Long and other members then adjusted the lane-keeping code so that, in situations like this where a blue cone line was missing, it would still follow the yellow line — improving performance.

Additionally using less severe track curvatures was found to improve performance helping understand the car limitations.
