# evaluation/

Experimental artefacts supporting the **Lazy Coulomb Planner (LCP)** paper.

This folder is only present on the `evaluation` branch. The planner source itself lives on `humble`.

## Layout

```
evaluation/
├── phase1/                      # Phase 1: environment setup verification
│   └── setup_verified.txt
├── phase2/                      # Phase 2: benchmark maps
│   ├── configs/
│   │   └── slam_mapping.yaml    # slam_toolbox config used to build the maps
│   ├── empty_lcp.launch.py      # launch file: TB3 burger + a chosen world
│   ├── worlds/                  # Gazebo .world sources for each map
│   │   ├── empty.world          # Map 1 — 22x22 m, sparse obstacles
│   │   ├── maze.world           # Map 2 — 20x25 m, narrow corridors + dead ends
│   │   ├── house.world          # Map 3 — copy of AWS RoboMaker small_house.world
│   │   └── event_venue.world    # Map 4 — 20x20 m, dense booth grid
│   ├── maps/                    # SLAMmed Nav2 maps (.pgm + .yaml)
│   │   ├── empty.{pgm,yaml}
│   │   ├── maze.{pgm,yaml}
│   │   ├── house.{pgm,yaml}
│   │   └── event_venue.{pgm,yaml}
│   └── screenshots/
│       └── empty.png
└── phase3/                      # Phase 3: benchmark harness deployment + smoke tests
    ├── README.md                # phase summary, results, reproducibility notes
    ├── smoke_test_navfn.csv     # NavFn × empty × 3 seeds
    ├── smoke_test_lcp.csv       # LazyCoulomb × empty × 3 seeds
    ├── smoke_seeds.yaml         # the (start, goal) pairs used by both smoke tests
    ├── smoke_path_check.png     # visual sanity check of paths over the empty map
    ├── visual_check.py          # script that produced smoke_path_check.png
    └── harness_changes/         # snapshot of files we created/edited in lcp_benchmark/
└── phase4/                      # Phase 4: LCP parameter tuning sweep
    ├── README.md                # phase summary, results, override rationale
    ├── chosen_config.yaml       # post-tuning nav2_params_lazy_coulomb.yaml
    ├── tuning_sweep.csv         # 270 runs x 24 columns
    ├── sweep_configs/           # the 27 generated config YAMLs
    └── tools/                   # generator, sweep driver, merge script
```

## Notes

- `house.world` references AWS RoboMaker residential models. To load it, the `aws-robomaker-small-house-world` package must be present in the workspace and built (so its `models/` directory is on `GAZEBO_MODEL_PATH`).
- Map 5 (`outdoor_park`) was scoped in the master document but later dropped during Phase 2 design review. The benchmark uses 4 maps.

## Paper

In preparation. Target venue: *Advanced Robotics* (Taylor & Francis) + parallel JOSS software paper.
