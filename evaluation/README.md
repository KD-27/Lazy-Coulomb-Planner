# evaluation/

Experimental artefacts supporting the **Lazy Coulomb Planner (LCP)** paper.

This folder is only present on the `evaluation` branch. The planner source itself lives on `humble`.

## Layout

```
evaluation/
├── phase1/                      # Phase 1: environment setup verification
│   └── setup_verified.txt
└── phase2/                      # Phase 2: benchmark maps
    ├── configs/
    │   └── slam_mapping.yaml    # slam_toolbox config used to build the maps
    ├── empty_lcp.launch.py      # launch file: TB3 burger + a chosen world
    ├── worlds/                  # Gazebo .world sources for each map
    │   ├── empty.world          # Map 1 — 22x22 m, sparse obstacles
    │   ├── maze.world           # Map 2 — 20x25 m, narrow corridors + dead ends
    │   ├── house.world          # Map 3 — copy of AWS RoboMaker small_house.world
    │   └── event_venue.world    # Map 4 — 20x20 m, dense booth grid
    ├── maps/                    # SLAMmed Nav2 maps (.pgm + .yaml)
    │   ├── empty.{pgm,yaml}
    │   ├── maze.{pgm,yaml}
    │   ├── house.{pgm,yaml}
    │   └── event_venue.{pgm,yaml}
    └── screenshots/
        └── empty.png
```

## Notes

- `house.world` references AWS RoboMaker residential models. To load it, the `aws-robomaker-small-house-world` package must be present in the workspace and built (so its `models/` directory is on `GAZEBO_MODEL_PATH`).
- Map 5 (`outdoor_park`) was scoped in the master document but later dropped during Phase 2 design review. The benchmark uses 4 maps.

## Paper

In preparation. Target venue: *Advanced Robotics* (Taylor & Francis) + parallel JOSS software paper.
