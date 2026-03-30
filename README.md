# benewake_lidar

ROS2 driver for Benewake LIDAR sensors that communicate over the MSOP (Multi-block Socket Protocol) via UDP.

Publishes `sensor_msgs/msg/LaserScan` messages. Supports single and dual LIDAR setups with optional remote sensor configuration through the sensor's built-in REST API.

> **Warning:** This software is provided as-is and has not been fully tested in production environments. Use at your own risk. No guarantees are made regarding its reliability, accuracy, or suitability for safety-critical applications. Always validate sensor data independently before relying on it for navigation or obstacle avoidance.

## Supported ROS2 Distributions

| Distribution | Ubuntu | Status |
|-------------|--------|--------|
| Humble | 22.04 | Supported |
| Jazzy | 24.04 | Supported |

## Architecture

```
LIDAR Sensor (UDP)
       |
       v
 +--------------+     +----------------+     +-------------------+
 | UdpReceiver  | --> | ScanAssembler  | --> | BenewakeLidarNode |
 | (boost::asio |     | (MSOP parsing, |     | (ROS2 publisher,  |
 |  IO thread)  |     |  scan assembly)|     |  parameters)      |
 +--------------+     +----------------+     +-------------------+
                                                      |
                                                      v
                                              /scan (LaserScan)
```

- **UdpReceiver** - Async UDP socket on a dedicated thread via `boost::asio`. Validates packet size (1206 bytes).
- **ScanAssembler** - Parses MSOP data blocks, detects 360-degree scan boundaries, builds `LaserScan` messages.
- **BenewakeLidarNode** - ROS2 node with `SensorDataQoS` publisher. Non-blocking constructor, clean shutdown.
- **SensorRestClient** - Optional HTTP client (`boost::beast` + `nlohmann/json`) for configuring the sensor via its REST API.

## Installation

### Prerequisites

```bash
sudo apt install libboost-system-dev nlohmann-json3-dev
```

### Build

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone <repository-url> benewake_lidar

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select benewake_lidar
source install/setup.bash
```

## Usage

### Single LIDAR

```bash
# Basic
ros2 launch benewake_lidar lidar1_scan.launch.py

# With RViz
ros2 launch benewake_lidar lidar1_scan_view.launch.py

# Custom parameters
ros2 launch benewake_lidar lidar1_scan.launch.py port:=2368 output_topic:=scan sensor_ip:=192.168.198.2
```

### Dual LIDAR

```bash
# Two sensors on different UDP ports
ros2 launch benewake_lidar lidar1_scan_dual_lidar.launch.py

# With RViz
ros2 launch benewake_lidar lidar1_scan_dual_lidar_view.launch.py

# Custom ports and topics
ros2 launch benewake_lidar lidar1_scan_dual_lidar.launch.py port_0:=2368 port_1:=2369 output_topic_0:=scan0 output_topic_1:=scan1
```

### Run Node Directly

```bash
ros2 run benewake_lidar benewake_lidar_scan_node --ros-args \
  -p host_ip:=0.0.0.0 \
  -p port:=2368 \
  -p output_topic:=scan \
  -p sensor_ip:=192.168.198.2
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `"laser"` | TF frame ID for the scan |
| `host_ip` | string | `"0.0.0.0"` | Local address to bind the UDP socket |
| `sensor_ip` | string | `"192.168.198.2"` | LIDAR sensor IP address (for REST API) |
| `port` | int | `2368` | UDP port to receive scan data |
| `sensor_http_port` | int | `80` | HTTP port for sensor REST API |
| `output_topic` | string | `"scan"` | Topic name for published LaserScan |
| `inverted` | bool | `false` | Reverse scan point order (for upside-down mounting) |
| `angle_offset` | int | `0` | Rotate the scan by this many degrees |
| `scan_freq` | string | `"30"` | Scan frequency in Hz (10, 20, 25, 30) |
| `filter` | string | `"3"` | On-sensor filter level (0-3) |
| `laser_enable` | string | `"true"` | Enable or disable the laser |
| `scan_range_start` | string | `"45"` | Scan start angle in degrees |
| `scan_range_stop` | string | `"315"` | Scan stop angle in degrees |
| `configure_sensor` | bool | `false` | Send configuration to the sensor via REST API on startup |

## Topics

### Published

| Topic | Type | QoS |
|-------|------|-----|
| `/scan` (configurable) | `sensor_msgs/msg/LaserScan` | SensorDataQoS (Best Effort, volatile) |

## Network Setup

The LIDAR sensor sends UDP packets to the host machine. Ensure network connectivity:

1. Connect the LIDAR sensor to the host via Ethernet
2. Configure the host network interface to be on the same subnet as the sensor (default: `192.168.198.x`)
3. The default data port is `2368` (second sensor uses `2369`)

```
Host (192.168.198.1)  <---Ethernet--->  Sensor (192.168.198.2)
         :2368 (UDP data)
         :80   (HTTP REST API)
```

## Sensor REST API

When `configure_sensor:=true`, the driver configures the sensor on startup via HTTP REST endpoints:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/v1/sensor/scanfreq` | PUT | Set scan frequency |
| `/api/v1/sensor/laser_enable` | PUT | Enable/disable laser |
| `/api/v1/sensor/scan_range/start` | PUT | Set scan start angle |
| `/api/v1/sensor/scan_range/stop` | PUT | Set scan stop angle |
| `/api/v1/sensor/filter` | PUT | Set filter level |
| `/api/v1/system/firmware` | GET | Read firmware information |
| `/api/v1/system/monitor` | GET | Read system metrics |
| `/api/v1/sensor/overview` | GET | Read current sensor configuration |

## Dependencies

| Dependency | Purpose |
|-----------|---------|
| `rclcpp` | ROS2 C++ client library |
| `sensor_msgs` | LaserScan message type |
| `boost::asio` | Async UDP socket |
| `boost::beast` | HTTP client for sensor REST API |
| `nlohmann/json` | JSON parsing for REST API responses |

## TODO

- [ ] Full hardware integration testing with Benewake LIDAR sensor
- [ ] Validate scan data accuracy and consistency across all supported scan frequencies
- [ ] Test dual LIDAR configuration
- [ ] Test sensor REST API configuration endpoints
- [ ] Verify graceful shutdown and reconnection behavior under network interruptions
- [ ] Test on both ROS2 Humble and Jazzy
- [ ] Remove the disclaimer once the above items are validated

## License

Apache-2.0
