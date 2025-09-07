# graph_pub – Wireless Data Publisher

This ROS 2 package publishes Wi-Fi (or other wireless) reference positions stored in an XML file as a **PointCloud2** and a **Marker** for easy visualization in RViz.  
It can also overlay the points on a map by reading the map’s origin and orientation from a standard `.yaml` map file.

---

## Features

- Reads positions (X, Y, Z, Quaternion) from an XML file.
- Applies static transform using map origin & orientation.
- Filters points that are too close to reduce clutter.
- Publishes:
  - `sensor_msgs/PointCloud2` on `/wireless_positions`
  - `visualization_msgs/Marker` on `/visualization_marker`
- Integrates easily with RViz for visualization.
- Launch file starts:
  - Map publisher node
  - Wireless data publisher node
  - RViz pre-configured session

---

## File Overview

- **`wireless_data_publisher.py`**  
  ROS 2 node implementing the logic to load, transform, filter, and publish points.

- **`launch/graph_pub.launch.py`**  
  Launches the wireless data publisher, map publisher, and RViz.

---

## Parameters

| Parameter          | Default                     | Description                                             |
|---------------------|-----------------------------|---------------------------------------------------------|
| `xml_file`          | `wireless_data_transform_lab.xml` | XML containing wireless positions.                      |
| `map_file`          | `mapaTransformSoloLab`       | Name of the map YAML file (without extension).          |
| `frame_id`          | `map`                        | Coordinate frame for the published messages.            |
| `publish_frequency` | `1.0` (Hz)                   | Rate at which to re-publish the static cloud & marker.   |
| `distance_threshold`| `3` (meters)                 | Minimum spacing between points (filters close ones).    |

---

## Topics

| Topic               | Type                       | Description                                  |
|----------------------|---------------------------|----------------------------------------------|
| `/wireless_positions`| `sensor_msgs/PointCloud2`  | Published positions with intensity.          |
| `/visualization_marker` | `visualization_msgs/Marker` | Line list marker connecting points (RViz). |

---

## Running the Node

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select graph_pub
source install/setup.bash
``` 

### 2. Launch with default parametes 
``` bash
ros2 launch launch/graph_pub.launch.py 
```

### Override parameters
``` bash
ros2 launch launch/graph_pub.launch.py  \
    map_file:=complete_lab \
    xml_file:=wireless_data_amcl.xml \
    frame_id:=map
```

