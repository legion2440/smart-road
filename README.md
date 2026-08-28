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

Run the optimized binary from:

```text
target/release/smart-road
```

## 📝 About

The simulation models a four-way intersection where every incoming direction has three dedicated movement lanes:

- right turn;
- straight;
- left turn.

A vehicle receives its route when it is spawned and never changes it. There are no traffic-light phases. Instead, approaching vehicles enter a controlled zone, request access to the intersection and adjust their speed according to the current reservations.

Several non-conflicting movements can cross at the same time. Conflicting movements are kept outside the conflict area until they can be admitted safely.

The simulation runs on a deterministic fixed physics timestep of `60 Hz`, while rendering is synchronized separately through SDL2 VSync when available.

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
| `Esc` | Exit and show final statistics |

Each manually spawned vehicle receives one of the three legal routes for its incoming direction. Spawn validation prevents a new vehicle from being placed on top of an existing one.

## 🧠 Smart intersection algorithm

The traffic controller is based on **movement reservations**, not traffic-light phases.

For every vehicle entering the smart-control area:

1. The vehicle is detected before the physical intersection.
2. Its immutable origin and route determine one of the 12 possible movements.
3. The controller checks a precomputed conflict matrix between all movement pairs.
4. Only the front vehicle of each waiting movement can request a reservation.
5. Candidates are ordered by waiting time and queue pressure.
6. A candidate receives a reservation only if its trajectory does not conflict with any currently reserved movement.
7. Several compatible movements may therefore cross simultaneously.
8. A reservation is released after the vehicle leaves the conflict area.

Waiting time prevents starvation, while queue pressure gives additional priority to movements that are accumulating traffic.

This keeps the controller independent from rendering and from the low-level vehicle physics.

## 🛡️ Safety model

Collision and proximity checks use **oriented bounding boxes (OBB)** and the **Separating Axis Theorem (SAT)**, so vehicles are checked according to their real heading rather than simple axis-aligned rectangles.

The safety model includes:

- positive safety margins around vehicle bodies;
- protected spawn positions;
- minimum following distance for vehicles on the same movement;
- conflict-area reservations for crossing trajectories;
- close-call detection;
- collision detection as an independent diagnostic metric.

Vehicles in the same lane are ordered by progress along their path. A follower cannot advance beyond the configured safe distance behind its leader.

## ⚙️ Vehicle model

Every vehicle tracks the required motion state directly:

- `time` — time spent under smart-intersection control;
- `distance` — distance travelled along its route;
- `velocity` — actual current velocity.

The controller uses three principal target speed levels:

```text
STOP    =   0 px/s
SLOW    =  50 px/s
CRUISE  = 120 px/s
```

Velocity does not jump instantly between these levels. Vehicles use bounded acceleration and braking:

```text
max acceleration = 100 px/s²
max braking      = 180 px/s²
```

A blocked vehicle slows while approaching the reservation boundary and stops if necessary. Once its movement is reserved, it accelerates back toward cruise speed.

## 📊 Statistics

Pressing `Esc` shows the final statistics window.

Required metrics:

- number of vehicles that passed the intersection;
- maximum velocity;
- minimum velocity;
- maximum traversal time;
- minimum traversal time;
- close calls.

Additional metrics are also collected:

- vehicles spawned;
- rejected spawn attempts;
- peak number of vehicles on the road;
- peak queue size in one lane;
- detected collisions;
- average controlled traversal time;
- average travelled distance.

The side panel also displays live traffic state while the simulation is running.

## 🎨 Visualization

The SDL2 renderer provides:

- a four-way road with three incoming movement lanes per direction;
- animated vehicle rotation along curved turning paths;
- vehicle sprites;
- blinking left/right turn signals;
- a live control and statistics panel;
- automatic logical scaling when the physical window size changes.

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
          | intersection     |            | lane following   |
          | reservation mgr  |            | + vehicle physics|
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

The main responsibilities are intentionally separated:

- `geometry` defines immutable paths and the movement conflict matrix;
- `controller` grants and releases smart-intersection reservations;
- `simulation` owns the fixed-timestep update and longitudinal vehicle motion;
- `collision` implements OBB/SAT geometry;
- `stats` collects final and live metrics;
- `render` and `sprites` contain presentation-only logic.

## 📁 Project structure

```text
smart-road/
├── .cargo/
│   └── config.toml
├── assets/
│   ├── cars.part1.b64
│   ├── cars.part2.b64
│   └── cars.part3.b64
├── src/
│   ├── collision.rs
│   ├── controller.rs
│   ├── geometry.rs
│   ├── main.rs
│   ├── render.rs
│   ├── simulation.rs
│   ├── sprites.rs
│   ├── stats.rs
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
- The green/red state in the side panel refers to simulation status; there are no traffic lights in the intersection.
- The project uses a fixed logical rendering area and scales it to the available window size.
- CMake 4 compatibility for the bundled SDL2 build is configured in `.cargo/config.toml`.

## 🧑‍💻 Author

- Nazar Yestayev (@nyestaye)
