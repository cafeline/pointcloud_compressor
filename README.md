# pointcloud_compressor: voxel dictionary compression for ROS 2 and CLI

`pointcloud_compressor` provides a reusable point cloud compression pipeline based on voxel grids and pattern dictionaries. The package offers both ROS 2 nodes for publishing compressed map data and a standalone CLI for offline workflows, enabling the same core compressor to be reused across robotics and non-ROS tooling.

## ROS 2 version

* ROS 2 Humble Hawksbill

## Quick start

### Install & Build
```bash
mkdir -p ros2_ws && cd ros2_ws
git clone <repository-url> src/pointcloud_compressor
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### CLI usage (runs without ROS 2)
```bash
# Compress
install/pointcloud_compressor/lib/pointcloud_compressor/pointcloud_compressor_cli \
  compress input.pcd output_prefix --voxel-size 0.01 --block-size 8

# Decompress
install/pointcloud_compressor/lib/pointcloud_compressor/pointcloud_compressor_cli \
  decompress output_prefix restored.pcd

# Search optimal settings
install/pointcloud_compressor/lib/pointcloud_compressor/pointcloud_compressor_cli \
  optimize input.pcd
```

## Compression artifacts and file formats

The CLI and ROS 2 node writes a HDF5 archive. 

| File | Groups / datasets (selected) | Notes |
| :--- | :--------------------------- | :---- |
| `compressed_map.h5` | `/metadata` (attributes: version, creation_time, frame_id, compression_method)<br>`/compression_params/voxel_size`, `dictionary_size`, `pattern_bits`, `block_size`, `block_index_bit_width`, `grid_origin`<br>`/dictionary/pattern_length`, `patterns`<br>`/compressed_data/block_indices` (bit-packed), `block_dims`, `point_count`<br>`/statistics` (compression ratio, bounding box, point counts) | Consolidates dictionary, indices, and grid statistics in a single container. Block indices are stored bit-packed; consumers should respect `block_index_bit_width` when decoding. |
| `raw_voxel_grid.h5` (when `save_raw_hdf5` is true) | `/raw_voxel_grid/voxel_values`, `occupied_voxels`, `dimensions`, `origin`, `voxel_size` | Stores the dense occupancy grid prior to compression for inspection or debugging. |

## Nodes

### pointcloud_compressor_node

Compresses a point cloud file and publishes a PatternDictionary message together with optional voxel markers for visualization. The node leverages the runtime compression library and does not subscribe to sensor streams. It is intended for preprocessing map assets.

#### Subscribed Topics

| Name | Type | Description |
| :--- | :--- | :---------- |
| _None_ | - | Files are loaded from disk; no ROS topic input is required. |

#### Published Topics

| Name | Type | Description |
| :--- | :--- | :---------- |
| `pattern_dictionary` | [`pointcloud_compressor/PatternDictionary`](msg/PatternDictionary.msg) | Detailed compression result including dictionary, indices, and statistics. |
| `occupied_voxel_markers` | [`visualization_msgs/MarkerArray`](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html) | CUBE_LIST markers of occupied voxels (enabled when `publish_occupied_voxel_markers` is true). |

#### Parameters

| Name | Type | Default | Description |
| :--- | :--- | :------ | :---------- |
| `input_file` | `string` | `""` | Path to the source point cloud (`.pcd`/`.ply`). |
| `input_pcd_file` | `string` | `""` | Legacy alias of `input_file`. |
| `voxel_size` | `double` | `0.01` | Edge length of voxels used during voxelization (meters). |
| `block_size` | `int` | `8` | Side length of cubic blocks grouped into dictionary patterns. |
| `use_8bit_indices` | `bool` | `false` | Forces 8-bit indices when feasible (otherwise auto-select). |
| `min_points_threshold` | `int` | `1` | Minimum number of points required for a voxel to be treated as occupied. |
| `publish_once` | `bool` | `true` | Publishes a single compressed result on startup when true. |
| `publish_interval_ms` | `int` | `1000` | Periodic publish interval (ms) when `publish_once` is false. |
| `publish_occupied_voxel_markers` | `bool` | `false` | Enables MarkerArray publication for visualization. |
| `save_hdf5` | `bool` | `false` | Writes compressed artifacts to an HDF5 map archive. |
| `hdf5_output_file` | `string` | `/tmp/compressed_map.h5` | Target path for the compressed HDF5 file. |
| `save_raw_hdf5` | `bool` | `false` | Exports the raw voxel occupancy grid to HDF5. |
| `raw_hdf5_output_file` | `string` | `/tmp/raw_voxel_grid.h5` | Output path for the raw voxel grid archive. |
| `bounding_box_margin_ratio` | `double` | `0.0` | Extra margin applied around the detected bounding box before voxelization. |

### block_size_optimizer_node

Evaluates compression performance for a range of block sizes and reports the best configuration. The node can execute once on startup or expose a trigger service for on-demand optimization, and optionally runs a final compression pass with the chosen block size.

#### Subscribed Topics

| Name | Type | Description |
| :--- | :--- | :---------- |
| _None_ | - | Reads the point cloud file specified by parameters. |

#### Published Topics

| Name | Type | Description |
| :--- | :--- | :---------- |
| `optimization_result` | [`std_msgs/String`](http://docs.ros.org/en/api/std_msgs/html/msg/String.html) | Human-readable summary of evaluated block sizes. |
| `optimal_block_size` | [`std_msgs/Int32`](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html) | Best block size discovered during optimization. |
| `best_compression_ratio` | [`std_msgs/Float32`](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html) | Compression ratio achieved by the optimal block size. |

#### Services

| Name | Type | Description |
| :--- | :--- | :---------- |
| `run_optimization` | [`std_srvs/Trigger`](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html) | Starts optimization when `run_once` is false. |

#### Parameters

| Name | Type | Default | Description |
| :--- | :--- | :------ | :---------- |
| `input_file` | `string` | `""` | Point cloud path to evaluate. |
| `min_block_size` | `int` | `4` | Minimum block size considered. |
| `max_block_size` | `int` | `32` | Maximum block size considered. |
| `step_size` | `int` | `1` | Step size for block size sweep. |
| `voxel_size` | `double` | `0.01` | Voxel size used during evaluation. |
| `verbose` | `bool` | `false` | Enables detailed logging of intermediate ratios. |
| `auto_compress` | `bool` | `false` | Runs a compression pass with the optimal block size after optimization. |
| `output_prefix` | `string` | `""` | Prefix for artifacts when `auto_compress` is true. |
| `run_once` | `bool` | `true` | Executes optimization immediately and shuts down upon completion. |
