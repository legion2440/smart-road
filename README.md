# Smart Road

A Rust/SDL2 simulation of an autonomous **smart intersection without traffic lights**. Vehicles follow dedicated right, straight and left lanes while a central reservation controller coordinates conflicting trajectories through speed control, safe spacing and movement priority.

· [Русская версия](README_RU.md)

## 📋 TOC

- [🚀 Quick start](#-quick-start)
- [📝 About](#-about)
- [🚗 Controls](#-controls)
- [🧠 Smart intersection algorithm](#-smart-intersection-algorithm)
- [🛡️ Safety model](#️-safety-model)
- [⚙️ Vehicle model](#️-vehicle-model)
- [📊 Statistics](#-statistics)
- [🧪 Tests and verification](#-tests-and-verification)
- [🎨 Visualization](#-visualization)
- [🏗️ Architecture](#️-architecture)
- [📁 Project structure](#-project-structure)
- [⚠️ Notes](#️-notes)
- [🧑‍💻 Author](#-author)

## 🚀 Quick start

### Requirements

- Rust `1.87+`
- Cargo
- CMake
- a C/C++ build toolchain
- Windows: MSVC / Visual Studio Build Tools

SDL2 is built through the Rust dependency with `bundled` and `static-link` features, so a separate SDL2 installation is not required.

### Clone

```bash
git clone https://github.com/legion2440/smart-road.git
cd smart-road
```

### Run

```bash
cargo run
```

The first build takes longer because the bundled SDL2 library is compiled together with the project.

### Release build

```bash
cargo build --release
```

The optimized binary is created under `target/release/`.

## 📝 About

The simulation models a four-way intersection where every incoming direction has three dedicated movement lanes:

- right turn;
- straight;
- left turn.

A vehicle receives its route when it is spawned and never changes it. There are no traffic-light phases. Approaching vehicles enter a controlled zone, request access to the intersection and adjust their speed according to active reservations.

Several non-conflicting movements can cross at the same time. Conflicting movements remain outside the conflict area until they can be admitted safely. Vehicles on the exact same movement may form a controlled convoy because longitudinal following distance is enforced separately.

The simulation uses a fixed `60 Hz` physics timestep. Rendering is synchronized independently through SDL2 VSync when available.

## 🚗 Controls

| Key | Action |
| --- | --- |
| `↑` | Spawn a vehicle travelling from South to North |
| `↓` | Spawn a vehicle travelling from North to South |
| `→` | Spawn a vehicle travelling from West to East |
| `←` | Spawn a vehicle travelling from East to West |
| `R` | Toggle continuous random vehicle generation |
| `Space` | Pause / resume simulation |
| `Backspace` | Reset the simulation |
| `Esc` | End the simulation and show final statistics |

Each manually spawned vehicle receives one of the three legal routes for its incoming direction. Spawn validation prevents a new vehicle from being placed on top of an existing one.

## 🧠 Smart intersection algorithm

The controller is based on **movement reservations**, not traffic-light phases.

1. A vehicle is detected before the physical intersection.
2. Its immutable origin and route determine one of 12 possible movements.
3. A precomputed conflict matrix describes which different movements overlap inside the conflict area.
4. The front waiting vehicle of each movement becomes a reservation candidate.
5. Candidates are ranked by waiting time and queue pressure.
6. Reservations are requested before the slow-down zone so an isolated vehicle does not brake unnecessarily.
7. A candidate is admitted when it does not conflict with any active different movement.
8. Vehicles on the same route can hold reservations concurrently; the following layer keeps their spacing safe.
9. A reservation is released after a vehicle leaves the conflict area.

Waiting time prevents starvation while queue pressure raises the priority of movements that are accumulating traffic.

## 🛡️ Safety model

Collision and proximity checks use **oriented bounding boxes (OBB)** and the **Separating Axis Theorem (SAT)**, so vehicle heading is part of the collision geometry.

The safety model includes:

- positive safety margins around vehicle bodies;
- protected spawn positions;
- a `66 px` minimum following distance for vehicles on the same movement;
- conflict-area reservations for crossing trajectories;
- close-call detection;
- collision detection as an independent diagnostic metric.

The `66 px` following distance is not an arbitrary constant: it is `30 px` vehicle length + `14 px` safety gap + `22 px` curvature allowance. The extra allowance keeps rotated OBBs separated on the tight `40 px` left-turn radius.

A follower normally stays behind its leader through braking-speed control. Position clamps remain only as emergency invariants: an unreserved vehicle cannot cross its stop boundary and a follower cannot cross the protected following limit.

## ⚙️ Vehicle model

Every vehicle directly tracks:

- `time` — time spent under smart-intersection control;
- `distance` — distance travelled along its route;
- `velocity` — actual current velocity.

The controller uses three principal target speed levels:

```text
STOP    =   0 px/s
SLOW    =  50 px/s
CRUISE  = 120 px/s
```

Velocity does not jump directly between controller targets. Each vehicle owns individual acceleration and braking limits derived from stable pseudo-random factors in the `0.80..1.25` range. The factors depend only on vehicle ID, so cars have different dynamics while deterministic scenarios remain reproducible.

Nominal dynamics are:

```text
acceleration = 100 px/s²
braking      = 180 px/s²
```

Before a stop boundary or a slower leader, the simulation applies a kinematic speed ceiling based on the vehicle's own braking capability:

```text
v_limit = sqrt(2 × braking × remaining_distance)
```

An additional `8 px` stopping margin keeps the normal braking path clear of the emergency position guards. This prevents followers from visually snapping from cruise speed to zero when a queue forms.

## 📊 Statistics

Pressing `Esc` ends the simulation and opens a final statistics screen inside the SDL window. The required labels use the assignment wording directly, including:

- `Max number of vehicles that passed the intersection`;
- `Max velocity`;
- `Min velocity`;
- `Max time that took a vehicle to pass the intersection`;
- `Min time that took a vehicle to pass the intersection`;
- `Close calls`.

Additional metrics:

- minimum moving velocity;
- vehicles spawned;
- rejected spawn attempts;
- peak number of vehicles on the road;
- real peak slow/stopped queue size in one lane;
- conservative peak number of vehicles on one approach;
- detected collisions;
- emergency safety-clamp activations;
- average controlled traversal time;
- average travelled distance.

The side panel also displays live traffic state during the simulation.

## 🧪 Tests and verification

Run the unit tests with:

```bash
cargo test
```

The suite covers:

- all 12 routes entering and leaving the intersection correctly;
- three distinct entry lanes per direction;
- symmetric conflict geometry with no self-conflict;
- protected spawn spacing;
- individual per-vehicle acceleration/braking profiles;
- an isolated vehicle crossing an empty intersection without unnecessary slow-down;
- a deterministic 60-second high-rate traffic soak scenario asserting zero collisions, zero close calls, zero emergency clamps, a real queue below 8 and a conservative approach load below 8.

For the visual/audit run, `R` should also be left enabled for at least one minute and the live `COLLISIONS`, `CLOSE CALLS` and queue behavior observed directly.

## 🎨 Visualization

The SDL2 renderer provides:

- a four-way road with three incoming movement lanes per direction;
- animated vehicle rotation along curved turning paths;
- vehicle sprites loaded from `assets/cars.bmp`;
- blinking left/right turn signals;
- a live control/statistics panel;
- an in-window final statistics screen that redraws while it is open;
- automatic logical scaling to the available window size.

Turning is represented by curved route geometry and continuous vehicle heading, so cars rotate through the maneuver instead of sliding sideways.

## 🏗️ Architecture

```text
                         +------------------+
                         |   keyboard / R   |
                         +---------+--------+
                                   |
                                   v
                         +------------------+
                         |    simulation    |
                         | fixed 60 Hz step |
                         +----+--------+----+
                              |        |
                   +----------+        +-----------+
                   |                               |
                   v                               v
          +------------------+            +------------------+
          | reservation mgr  |            | lane following   |
          | conflict matrix  |            | vehicle physics  |
          +--------+---------+            +---------+--------+
                   |                                |
                   +---------------+----------------+
                                   |
                                   v
                         +------------------+
                         | geometry + OBB   |
                         | conflict checks  |
                         +---------+--------+
                                   |
                         +---------+---------+
                         |                   |
                         v                   v
                  +-------------+     +-------------+
                  | statistics  |     | SDL renderer|
                  +-------------+     +-------------+
```

Main responsibilities:

- `geometry` defines immutable paths, safety constants and the conflict matrix;
- `controller` grants and releases reservations;
- `simulation` owns the fixed-timestep update, kinematic braking and longitudinal following;
- `vehicle` stores per-car motion and braking characteristics;
- `collision` implements OBB/SAT geometry;
- `stats` collects runtime and final metrics;
- `stats_screen` renders the final report;
- `render`, `sprites` and `ui_font` contain presentation logic.

## 📁 Project structure

```text
smart-road/
├── .cargo/
│   └── config.toml
├── assets/
│   ├── README.md
│   └── cars.bmp
├── src/
│   ├── collision.rs
│   ├── controller.rs
│   ├── geometry.rs
│   ├── main.rs
│   ├── render.rs
│   ├── simulation.rs
│   ├── sprites.rs
│   ├── stats.rs
│   ├── stats_screen.rs
│   ├── ui_font.rs
│   └── vehicle.rs
├── .gitignore
├── Cargo.toml
├── README.md
└── README_RU.md
```

## ⚠️ Notes

- Arrow keys describe the **travel direction**, not the side where the vehicle appears.
- `R` is a toggle: one press starts continuous random generation and the next press stops it.
- The colored state in the side panel describes the simulation; there are no traffic lights.
- CMake 4 compatibility for the bundled SDL2 build is configured in `.cargo/config.toml`.
- The controller is an idealized autonomous-intersection model: it assumes perfect route knowledge and does not model sensor uncertainty or communication latency.

## 🧑‍💻 Author

- Nazar Yestayev (@nyestaye)
