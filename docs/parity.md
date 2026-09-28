# Java MapleSim parity

## Included

- Physics bodies, fixtures, shapes and world management
- Arena ownership, timing, scoring and game-piece lifecycle
- Swerve drivetrain, module and gyro simulation
- Motor, battery and motor-controller simulation
- Intake and projectile simulation
- Field mirroring and math utilities
- 2026 REBUILT arena, fuel, hub and outpost behavior

## Not yet ported

- `SelfControlledSwerveDriveSimulation`
- `ArenaEvergreen`
- The 2024 CRESCENDO season package
- The 2025 REEFSCAPE season package
- `LegacyFieldMirroringUtils2024`
- One-module swerve configurations; the C++ implementation currently requires four modules

`Goal::withRotationTolerance()` is intentionally absent because the pinned Java method is deprecated and has no effect. There is no core MapleSim auto-chooser API missing from the pinned source tree.

## Intentional C++ behavior changes

- `SimMotorConfigs::CalculateVoltage` converts current to torque before asking WPILib's motor model for voltage.
- Custom swerve module translations validate the replacement translations.
- Drive-base radius is the half-diagonal.
- Game-piece queries actually filter by type.
- Invalid total-current values are rejected before battery simulation.
- Removed game pieces remain alive until the end of the physics sub-tick so duplicate Box2D contact callbacks cannot access freed memory.
- Arena collections own drivetrains, intakes, game pieces and custom simulations because C++ does not have Java garbage collection.
- Random sources can be seeded for deterministic scenarios.

## Physics-engine differences

- Box2D uses `float` internally while WPILib and MapleSim-facing math use `double`.
- Contact management, warm starting and restitution are behaviorally similar to dyn4j but not trace-identical.
- Box2D polygon skin makes collision begin roughly 1 cm earlier.
- Damping and accumulated forces use dyn4j's formula before each Box2D step; Box2D's own damping is disabled.
- Thin intake rectangles bypass Box2D hull welding.
- Box2D keeps its stock 0.5-second compile-time sleep threshold instead of the Java port's 0.02-second dyn4j setting.
- Reefscape-specific Coral and Algae stacking logic is absent with the 2025 season package.
