# MapleSim C++

An unofficial native C++ port of [MapleSim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) for WPILib robot simulation. It uses Box2D 2.4.2 and packages the library as a GradleRIO C++ vendordep.

This is a personal project. It is not affiliated with or endorsed by Shenzhen Robotics Alliance, Iron Maple or my team (254).

## What is included

- A two-dimensional rigid-body world with collision handling
- Swerve drivetrain, module and gyro simulation
- An optional self-controlled swerve layer with module control and pose estimation
- Motor-controller, mechanism and battery simulation
- Intakes, projectiles and field game pieces
- Scoring and match-state support
- An empty Evergreen arena with optional wall segments
- The 2026 REBUILT arena, hubs, outposts and fuel

## Usage

### Build and publish

Clone the repository with its Box2D submodule and run the release build:

```bash
git clone --recurse-submodules https://github.com/thanewye/maple-sim-cpp.git
cd maple-sim-cpp
./gradlew build -PreleaseMode
```

Create the Maven repository and expanded vendordep JSON with the final version, group and public URLs:

```bash
./gradlew clean build publish -PreleaseMode \
  -PpublishVersion=0.1.0 \
  -PpublishGroup=com.example.maplesim \
  -PvendordepMavenUrl=https://example.github.io/maple-sim-cpp/maven \
  -PvendordepJsonUrl=https://example.github.io/maple-sim-cpp/MapleSimCpp.json
```

The Maven repository is written to `build/repos/releases`, and the generated vendordep is written to `build/vendordep/MapleSimCpp.json`. Replace the example coordinates and URLs before publishing. See [PUBLISHING.md](PUBLISHING.md) for the complete release process.

### Use in robot code

The arena owns the simulations registered with it and advances them from `SimulationPeriodic()`:

```cpp
#include <memory>

#include <frc/geometry/Pose2d.h>

#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/simulation/drivesims/SwerveDriveSimulation.h"
#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

namespace simulation = maplesim::simulation;
namespace drivesims = simulation::drivesims;

auto& arena = simulation::SimulatedArena::GetInstance();
auto& drive = arena.AddDriveTrainSimulation(std::make_unique<drivesims::SwerveDriveSimulation>(
    drivesims::configs::DriveTrainSimulationConfig::Default(), frc::Pose2d{}));

arena.SimulationPeriodic();
```

A robot project still needs to connect the simulated modules and gyro to its hardware abstraction layer.

## Documentation

- [Java MapleSim parity and known differences](docs/parity.md)
- [Architecture and lifetime rules](docs/architecture.md)
