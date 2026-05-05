"""
Smoke test: 3 runs of NavFn on the empty map.

Use this to verify the harness works end-to-end before scaling up.
"""

import os
import subprocess
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction


def expand(s: str) -> str:
    return os.path.expandvars(os.path.expanduser(s))


def launch_setup(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory
    pkg_share = Path(get_package_share_directory('lcp_benchmark')) / 'config'

    map_yaml = expand('${HOME}/lcp_paper_ws/maps/empty.yaml')
    out_dir = Path(expand('${HOME}/lcp_paper_ws/results'))
    out_dir.mkdir(parents=True, exist_ok=True)
    output_csv = str(out_dir / 'smoke_test_lcp.csv')
    seeds_file = str(out_dir / 'smoke_seeds.yaml')

    # Generate just 3 seeds
    subprocess.run([
        'ros2', 'run', 'lcp_benchmark', 'generate_seeds',
        map_yaml, seeds_file, '3', '0',
    ], check=True)

    # Bring up map + planner
    bringup = Path(get_package_share_directory('lcp_benchmark')) / 'launch' / 'lcp_planner_only.launch.py'
    params_file = str(pkg_share / 'nav2_params_lazy_coulomb.yaml')
    bringup_proc = subprocess.Popen([
        'ros2', 'launch', str(bringup),
        f"params_file:={params_file}",
        f"map:={map_yaml}",
    ])
    time.sleep(8.0)

    try:
        subprocess.run([
            'ros2', 'run', 'lcp_benchmark', 'harness',
            '--ros-args',
            '-p', f"output_csv:={output_csv}",
            '-p', f"planner_name:=LazyCoulomb",
            '-p', f"map_name:=empty",
            '-p', f"map_yaml:={map_yaml}",
            '-p', f"seeds_file:={seeds_file}",
            '-p', f"plan_timeout_s:=30.0",
        ], check=False)
    finally:
        bringup_proc.terminate()
        try:
            bringup_proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            bringup_proc.kill()

    print(f"\n=== Smoke test done. Check {output_csv} for 3 rows ===")
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
