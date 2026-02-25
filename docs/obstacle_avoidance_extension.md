# Obstacle Avoidance Extension — smooth_nav

This document describes how to extend the `smooth_nav` system to handle obstacles, using the existing Strategy pattern architecture.

---

## Approach: Local Re-planning

The cleanest integration adds a **reactive obstacle avoidance layer** between the global trajectory and the controller, without modifying existing packages.

### New Package: `smooth_nav_avoidance`

```
src/smooth_nav_avoidance/
├── include/smooth_nav_avoidance/
│   ├── i_obstacle_detector.hpp        # Interface
│   ├── lidar_obstacle_detector.hpp    # LaserScan-based
│   ├── i_local_planner.hpp            # Interface
│   ├── vfh_planner.hpp                # Vector Field Histogram
│   └── dwa_planner.hpp                # Dynamic Window Approach
├── src/
│   ├── lidar_obstacle_detector.cpp
│   ├── vfh_planner.cpp
│   ├── dwa_planner.cpp
│   └── avoidance_node.cpp
├── config/avoidance.yaml
├── launch/avoidance.launch.py
├── CMakeLists.txt
└── package.xml
```

### Integration with Existing Architecture

1. **Subscribe** to `/scan` (LaserScan) for obstacle detection
2. **Subscribe** to the reference trajectory from the tracker
3. **Publish** modified `/cmd_vel` that avoids obstacles while tracking the trajectory
4. Uses the **Strategy pattern** — swap between VFH, DWA, or potential fields via YAML config

### Algorithm Options

| Algorithm | Pros | Cons |
|-----------|------|------|
| **VFH** (Vector Field Histogram) | Fast, works well in corridors | Oscillation in open spaces |
| **DWA** (Dynamic Window Approach) | Considers robot dynamics | Computationally heavier |
| **APF** (Artificial Potential Fields) | Simple to implement | Local minima issues |

### Minimal Integration

The simplest approach uses a "safety wrapper" node:

```cpp
// Pseudocode for avoidance_node
void controlCallback() {
    auto scan = getLatestScan();
    auto cmd = getLatestCmdVel();  // from trajectory_tracker
    
    if (isObstacleAhead(scan, safety_distance_)) {
        cmd = localPlanner_->avoid(scan, cmd);
    }
    
    cmd_vel_pub_->publish(cmd);
}
```

### Configuration

```yaml
avoidance:
  safety_distance: 0.35        # meters
  planner_type: "vfh"          # vfh, dwa, apf
  max_deviation: 0.5           # max deviation from trajectory
  enable_avoidance: true       # toggle on/off
```

---

## Why This Extends Cleanly

- **No changes to existing packages** — avoidance is a separate node
- **Strategy pattern reused** — `ILocalPlanner` mirrors `IPathSmoother`, `IController`
- **Config Over Code** — enable/disable and tune via YAML
- **Testable separately** — unit test planners with synthetic scan data
