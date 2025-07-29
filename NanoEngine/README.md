# NanoEngine - A very simple 3D Physics Engine in F#

NanoEngine is a 3D rigid-body physics engine written entirely in F#.

The engine simulates a world of oriented bounding boxes (OBBs) within a unique, spatially-hashed environment based on a grid of hexagonal prisms. A key feature is its toroidal (wrapping) world, where objects moving off one edge seamlessly reappear on the opposite side.

The project includes a comprehensive test suite that demonstrates complex physics interactions and validates the engine's correctness, especially at the wrapping world boundaries.

## Core Features

*   **3D Rigid-Body Simulation:** Simulates position, velocity, and orientation for box-shaped objects.
*   **Unique Grid System:** The world is partitioned using a grid of **hexagonal prisms**, which are further subdivided into 12 triangular sub-prisms. This forms the basis for the spatial hash system.
*   **Toroidal (Wrapping) World:** The simulation space wraps around on the X and Y axes, creating a seamless, infinite world.
*   **Three-Phase Collision Detection:**
    *   **Broad Phase:** A highly efficient **Spatial Hash** based on the hexagonal prism grid to quickly find potentially colliding bodies.
    *   **Mid-Phase:** An Axis-Aligned Bounding Box (AABB) check is performed on potential pairs from the broad phase. This is a quick rejection step that filters out many pairs before the more expensive narrow-phase check.
    *   **Narrow Phase:** For pairs that pass the AABB check, precise collision detection is performed using the **Separating Axis Theorem (SAT)** for Oriented Bounding Boxes (OBBs).
*   **Advanced Sleeping System:** An **island-based sleeping system** dramatically improves performance by deactivating groups of objects that have come to rest. The engine intelligently manages waking, sleeping, merging, and splitting these islands.
*   **Kinematic and Dynamic Interactions:** Supports standard dynamic objects, static geometry, and special entities like destructible flora (trees) and friction-inducing bushes.
*   **Complex Scenarios:** Capable of handling complex behaviors like falling-over animations, grid-snapping for stable placement, and basic cellular automata for liquids.

---

## Architectural Highlights & Nuances

This engine is not a universal physics library, but rather an exploration of the possibility of developing a physics engine that works correctly at the seams of a toroidal world.

### 1. Simple-First Design

The engine was developed as an experiment for further transfer to a ECS execution infrastructure and further multi-threaded optimization.

### 2. Hexagonal Prism Grid

Instead of a traditional cubic grid, NanoEngine uses a more complex world discretization:
*   The XY plane is tiled with hexagons.
*   The Z axis is divided into layers, creating hexagonal prisms.
*   Each prism is subdivided into 12 smaller **triangular prisms**. This fine-grained subdivision forms the key for the spatial hash, allowing for very precise broad-phase collision culling.

This choice allows for more isotropic (directionally uniform) spatial queries on the horizontal plane compared to a square grid.

### 3. Robust Island Management (`Island` module)

The sleeping system is one of the most complex and critical parts of the engine.
*   **Islands:** Groups of interacting bodies are tracked as "islands."
*   **Sleeping:** An entire island is put to sleep if all its bodies have low velocity for a set number of frames and are resting on static geometry. Sleeping bodies are moved to a separate "sleeping" spatial hash and are not processed in the main physics loop.
*   **Waking:** An island wakes up if a dynamic object collides with one of its sleeping members.
*   **Merging & Splitting:** The engine constantly manages the topology of islands. Islands are merged when their bodies come into contact and are split when a group of bodies loses contact with the rest of its island. This is essential for maintaining simulation correctness.

---

## Strengths

*   **Seamless Wrapping World:** The logic for handling interactions across the world's edges is robust and thoroughly tested, making it suitable for games or simulations requiring this feature.
*   **Stable and Predictable:** The impulse-based solver and friction model provide stable interactions, even in complex stacking scenarios.
*   **Extensive Test Suite:** The `Tests` module is not just for verification; it serves as a powerful demonstration of the engine's capabilities in a variety of challenging, multi-stage scenarios.

## Limitations & Weaknesses

*   **OBB-Only Collision Shapes:** The engine is hard-coded to only support Oriented Bounding Boxes (cuboids). It does not support other primitive shapes like spheres, capsules, or complex mesh colliders.
*   **Simplified Rotational Physics:** The engine tracks orientation but does not simulate full rotational dynamics. It lacks concepts like **angular velocity, torque, and inertia tensors**. Rotations are primarily handled kinematically for scripted events like falling over, rather than emerging from physical forces.
*   **Niche Application:** The unique combination of a hexagonal grid and a wrapping world makes it highly specialized. Adapting it to a standard 3D environment would require significant work.
*   **The basic mechanics are well optimized (collision calculations)**, but further optimization is needed.
*   **The code is in places frankly sloppy in terms of architecture (ideally, all subsystems should be hidden behind a common facade)**, but due to the planned transfer of the engine's basic algorithms to ECS architecture, this was not done.
  
## Code Overview

*   `Engine.fs`: The main file containing all modules.
    *   `Engine`: Defines core constants and foundational types (`Vector3`, `Matrix3x3`).
    *   `Grid`: Handles all logic for converting between world coordinates and the hexagonal prism grid.
    *   `Body`: Defines the `Body.T` type for dynamic objects.
    *   `SpatialHash`: The broad-phase collision detection system.
    *   `Collision`: The **mid-phase (AABB) and narrow-phase (SAT)** collision detection algorithms.
    *   `Island`: The crucial module for managing sleeping, waking, merging, and splitting groups of bodies.
    *   `Simulation`: The main simulation loop orchestrator. It integrates all other systems to advance the physics state.
    *   `Flora`, `Geometry`, `Liquid`: Modules for managing static world elements.
*   `Tests.fs`: Contains the test runner and a comprehensive, multi-stage test definition that puts the engine through its paces.

## How to Run

The project is a console application whose `EntryPoint` is the test runner. To see the engine in action, simply build and run the project. It will execute a series of complex physics tests at various locations within the toroidal world (center, edges, corners) and report the results.
