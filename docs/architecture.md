# Architecture and invariants

```text
Consumer robot simulation
        |
        v
MapleSim C++
  drivetrain and motor sims, game pieces, arena, intake, goals and REBUILT 2026
        |
        v
maplesim::physics
  WPILib-typed Body, Fixture, Shape, World and Vector2d wrapper
        |
        v
Box2D 2.4.2
```

The physics wrapper exists because Box2D owns `b2Body` allocation and its bodies cannot be subclassed. The wrapper preserves MapleSim's inheritance model while keeping Box2D types out of public headers. It accepts WPILib geometry and units directly, avoiding dyn4j-style conversion calls throughout the library.

Each normal robot period is split into five 4 ms arena sub-ticks. A sub-tick updates the battery, drivetrains, motor-controller simulation state, projectiles, Box2D world, intakes and field mechanisms.

## Ownership and lifetime

The arena owns every drivetrain, intake, game piece and custom simulation. Consumer code holds references or non-owning pointers into those objects.

Arena members must be declared in this order so reverse C++ destruction remains safe:

1. Physics world
2. Drivetrains
3. Intakes
4. Game pieces
5. Custom simulations

Additional invariants:

- Removed game pieces remain alive until the end of the sub-tick.
- `Body`, `Fixture` and `ContactSubscription` detach safely regardless of destruction order.
- `SimulatedArena::GetInstance()` intentionally leaks the singleton so references remain valid and NetworkTables publishers do not outlive ntcore.
- `OverrideInstance` must run before registering drivetrains or intakes.
- The arena uses `std::recursive_mutex` because Java's `synchronized` is re-entrant and reset operations lock recursively.
- Bodies and fixtures must not be created or destroyed inside a Box2D contact callback.
- Cross-translation-unit statics use function-local accessors to avoid initialization-order bugs.
