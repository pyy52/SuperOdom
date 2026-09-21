# Super Odometry Codespace Migration Package

This directory contains reference materials collected from the local development environment for the **GitHub Codespace Agent**.

## Primary Reference Document
- **[`codespace_agent_local_reference.md`](./codespace_agent_local_reference.md)**: **Start here!** Contains the previous working setup, compiler requirements, memory constraints, known working commands, and dataset expectations.

## Reference Files
- **`container_relevant_apt_packages.txt`**: List of installed apt packages from the previous working Ubuntu 22.04 + ROS 2 Humble container.
- **`local_colcon_list.txt`**: List of the 7 packages comprising this workspace.
- **`local_project_key_files.txt`**: Key launch, config, and build files.
- **`local_env_report.md`**: Detailed host machine hardware and environment report.
- **`local_dataset_manifest.txt`**: List and sizes of local datasets.
- **`local_dataset_size_summary.txt`**: Summary of dataset disk sizes.

## Key Rules for Codespace Agent
1. The target environment is **Ubuntu 22.04 + ROS 2 Humble**.
2. When compiling with `colcon`, always use `--parallel-workers 1` or `--parallel-workers 2` to prevent memory exhaustion during Ceres/GTSAM template compilation:
   ```bash
   colcon build --packages-select super_odometry super_odometry_vio --parallel-workers 1
   ```
3. Dynamic library path: ensure `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`.
4. Large datasets should be placed outside the git repository under `/workspaces/data/bags/`.
