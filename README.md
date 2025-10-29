# vq_occupancy_compressor: lossless vector-quantized 3D occupancy grid compression for ROS 2 and CLI

This program performs lossless compression of occupancy grid maps, allowing the compressed map data to be used directly. The maps are compressed using vector quantization. The compressed map retains binary occupied/free data and does not include unknown cells. As input, it supports binary PCD and PLY files, and it runs both on ROS 2 and in a standalone CLI environment.


## ROS 2 version

* ROS 2 Humble Hawksbill

## Quick start

### Installation & Usage

#### ROS2
```bash
mkdir -p ros2_ws && cd ros2_ws
git clone <repository-url> src/vq_occupancy_compressor
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vq_occupancy_compressor vq_occupancy_compressor.launch.py
```

#### CLI
```bash
git clone <repository-url>
cd vq_occupancy_compressor
cmake -S . -B build -DBUILD_TESTING=OFF
cmake --build build --target vq_occupancy_compressor_cli
./build/vq_occupancy_compressor_cli compress path/to/vq_occupancy_compressor_params.yaml
```

The CLI expects a single YAML configuration file. The layout matches the ROS 2 parameter files used by the nodes:

```yaml
# config/vq_occupancy_compressor_params.yaml
vq_occupancy_compressor_node:
  ros__parameters:
    input_file: /absolute/path/to/map.pcd
    voxel_size: 0.01
    block_size: 8
    save_hdf5: true
    hdf5_output_file: /tmp/compressed_map.h5
    save_raw_hdf5: false
```
To run block-size optimisation from the CLI, use the optimisation YAML. The optimize command automatically performs a compression pass using the discovered settings:

```bash
./build/vq_occupancy_compressor_cli optimize path/to/block_size_optimizer.yaml
```

The same files can be re-used with the ROS 2 launch configuration.

## Compression artifacts and file formats

The CLI and ROS 2 node writes a HDF5 archive.

| File | Groups / datasets (selected) | Notes |
| :--- | :--------------------------- | :---- |
| `compressed_map.h5` | `/metadata` (attributes: version, creation_time, frame_id, compression_method)<br>`/compression_params/voxel_size`, `dictionary_size`, `pattern_bits`, `block_size`, `block_index_bit_width`, `grid_origin`<br>`/dictionary/pattern_length`, `patterns`<br>`/compressed_data/block_indices` (bit-packed), `block_dims`, `point_count`<br>`/statistics` (compression ratio, bounding box, point counts) | Consolidates dictionary, indices, and grid statistics in a single container. Block indices are stored bit-packed; consumers should respect `block_index_bit_width` when decoding. |
| `raw_voxel_grid.h5` (when `save_raw_hdf5` is true) | `/raw_voxel_grid/voxel_values`, `occupied_voxels`, `dimensions`, `origin`, `voxel_size` | Stores the dense occupancy grid prior to compression for inspection or debugging. |

## Nodes

### vq_occupancy_compressor_node

Compresses a point cloud file and publishes a PatternDictionary message together with optional voxel markers for visualization. The node leverages the compression bridge library and does not subscribe to sensor streams. It is intended for preprocessing map assets.


#### Subscribed Topics

| Name | Type | Description |
| :--- | :--- | :---------- |
| _None_ | - | Files are loaded from disk; no ROS topic input is required. |

#### Published Topics

| Name | Type | Description |
| :--- | :--- | :---------- |
| `pattern_dictionary` | [`vq_occupancy_compressor/PatternDictionary`](msg/PatternDictionary.msg) | Detailed compression result including dictionary, indices, and statistics. |
| `occupied_voxel_markers` | [`visualization_msgs/MarkerArray`](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html) | CUBE_LIST markers of occupied voxels (enabled when `publish_occupied_voxel_markers` is true). |

#### Parameters

| Name | Type | Default | Description |
| :--- | :--- | :------ | :---------- |
| `input_file` | `string` | `""` | Path to the source point cloud (`.pcd`/`.ply`). |
| `voxel_size` | `double` | `0.01` | Edge length of voxels used during voxelization (meters). |
| `block_size` | `int` | `8` | Side length of cubic blocks grouped into dictionary patterns. |
| `min_points_threshold` | `int` | `1` | Minimum number of points required for a voxel to be treated as occupied. |
| `publish_occupied_voxel_markers` | `bool` | `false` | Enables MarkerArray publication for visualization. |
| `save_hdf5` | `bool` | `false` | Writes compressed artifacts to an HDF5 map archive. |
| `hdf5_output_file` | `string` | `/tmp/compressed_map.h5` | Target path for the compressed HDF5 file. |
| `save_raw_hdf5` | `bool` | `false` | Exports the raw voxel occupancy grid to HDF5. |
| `raw_hdf5_output_file` | `string` | `/tmp/raw_voxel_grid.h5` | Output path for the raw voxel grid archive. |

### block_size_optimizer_node

Evaluates compression performance for a range of block sizes and reports the best configuration. The node can execute once on startup or expose a trigger service for on-demand optimization, and always runs a final compression pass using the optimal block size.

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
| `run_once` | `bool` | `true` | Executes optimization immediately and shuts down upon completion. |
