FROM osrf/ros:jazzy-desktop-full-noble
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-slam-toolbox \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
# Python dependencies of Nav2's tools/planner_benchmarking scripts
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-transforms3d python3-seaborn python3-tabulate python3-scipy \
    && rm -rf /var/lib/apt/lists/*
