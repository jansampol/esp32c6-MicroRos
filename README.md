# MAMRI firmware — ESP32-C6 and micro-ROS

This firmware connects the MAMRI robot controller to ROS 2 over Wi-Fi using micro-ROS. It receives joint paths and controller commands, drives the pneumatic steppers, and provides robot state feedback.

This guide covers the ESP-IDF 5.5 workflow used for this project. Run all project commands from the repository root (the directory containing `platformio.ini`), not from `main/`.

## Requirements

- An ESP32-C6 board; the PlatformIO configuration targets `esp32-c6-devkitc-1`.
- A USB data cable and the appropriate USB serial driver for your board.
- ESP-IDF **5.5**, with its ESP32-C6 toolchain installed.
- Python 3 and the micro-ROS build dependencies listed below.
- A ROS 2 host with a compatible micro-ROS Agent installed, reachable from the board over Wi-Fi.
- The MAMRI controller hardware for robot operation.

The micro-ROS ESP-IDF component is included under `components/micro_ros_espidf_component/`. Its [README](../components/micro_ros_espidf_component/README.md) contains additional dependency and Agent setup instructions.

## 1. Prepare your terminal

Replace the paths below with your local repository and ESP-IDF installation paths:

```bash
cd /path/to/Mamri_v6_PlatformIO

# Activate the project environment if you use one.
# Create it once with: python3 -m venv .venv
source .venv/bin/activate

# Load ESP-IDF and its Python environment.
source /path/to/esp-idf-v5.5/export.sh
idf.py --version
```

If you do not use a project `.venv`, skip its activation. ESP-IDF exports its own Python environment. Install the micro-ROS dependencies after exporting ESP-IDF so they are available to the build:

```bash
python -m pip install catkin_pkg lark-parser colcon-common-extensions
```

Repeat the environment activation and ESP-IDF export in each new build terminal. Use a separate terminal for ROS 2; the bundled micro-ROS component recommends building without sourcing a ROS 2 setup script in the build shell.

## 2. Configure the firmware

The repository defaults select `esp32c6`. Open the configuration menu:

```bash
idf.py menuconfig
```

Under **micro-ROS Settings**, configure:

| Setting | Value |
| --- | --- |
| micro-ROS middleware | micro-ROS over eProsima Micro XRCE-DDS |
| micro-ROS network interface select | WLAN interface |
| WiFi Configuration → WiFi SSID | Your Wi-Fi network name |
| WiFi Configuration → WiFi Password | Your Wi-Fi password |
| micro-ROS Agent IP | The reachable IPv4 address of the Agent host |
| micro-ROS Agent Port | `8888`, or the port used by your Agent |

Use your own network settings; the checked-in values are specific to the development setup. The Agent address must be reachable from the ESP32, so do not use `localhost` or `127.0.0.1`.

### Required SHA settings

The working ESP-IDF 5.5 setup requires hardware SHA acceleration to remain disabled and SHA1 support to remain enabled. Under **Component config → mbedTLS**, locate these options (use menuconfig search if needed):

- **Enable hardware SHA acceleration:** disabled.
- **SHA1:** enabled.

Save and exit. Confirm that `sdkconfig` contains:

```text
CONFIG_IDF_TARGET="esp32c6"
CONFIG_MBEDTLS_SHA1_C=y
# CONFIG_MBEDTLS_HARDWARE_SHA is not set
```

`sdkconfig.defaults` already disables hardware SHA acceleration. Check the active `sdkconfig` as well, especially when reusing an existing configuration.

## 3. Build

```bash
idf.py build
```

For a clean rebuild when resolving stale build files:

```bash
idf.py fullclean
idf.py build
```

A full clean is not needed for every source change. If a copied checkout contains a build directory from another machine, clear that generated directory before building locally.

## 4. Flash the board

Connect the board and identify its serial port. On macOS:

```bash
ls /dev/cu.*
```

On Linux:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Use the port that belongs to your board. For example, on macOS:

```bash
idf.py -p /dev/cu.usbserial-140 flash
```

Replace `/dev/cu.usbserial-140` with your actual port. Rebuild and flash after changing firmware or network settings.

## 5. Connect to ROS 2

In a separate terminal on the ROS 2 host, source your ROS 2 installation and the workspace containing the micro-ROS Agent. Start the installed Agent with the same UDP port configured in the firmware:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Keep the Agent running while using the robot. Ensure the network allows traffic between the ESP32 and the host, including the configured UDP port.

In another ROS 2 terminal, check discovery:

```bash
ros2 node list
ros2 node info /esp32_path_subscriber
ros2 topic list
```

The firmware defines these interfaces:

| Topic | Message type | Firmware role |
| --- | --- | --- |
| `/mamri/planning/joint_path_execute` | `std_msgs/msg/Float64MultiArray` | Receives joint paths |
| `/mamri/esp32/cmd` | `std_msgs/msg/String` | Receives controller commands |
| `/mamri/esp32/state` | `std_msgs/msg/String` | Publishes robot state responses |

See [MicroRosManager.cpp](MicroRosController/MicroRosManager.cpp) for the ROS interfaces and [app_main.cpp](app_main.cpp) for command handling.

## Everyday workflow

After completing the initial setup, use this sequence from a new terminal, substituting your paths and serial port:

```bash
cd /path/to/Mamri_v6_PlatformIO
source .venv/bin/activate  # If using a project virtual environment
source /path/to/esp-idf-v5.5/export.sh
idf.py build
idf.py -p /dev/cu.usbserial-140 flash
```

Run the micro-ROS Agent on the ROS 2 host as described above.

## Troubleshooting

| Problem | What to check |
| --- | --- |
| `idf.py` is not found | Export ESP-IDF in the current terminal. |
| Missing `colcon` or Python packages | Install the build dependencies after exporting ESP-IDF. |
| Build refers to another machine's paths | Clear the old generated build directory and rebuild locally. |
| Board cannot be flashed | Check the cable, serial driver, port, and whether another application is using the port. |
| ESP32 does not appear in ROS 2 | Check Wi-Fi credentials, Agent IP and port, Agent availability, and network isolation or firewall settings. |
| SHA-related build or runtime problems | Confirm hardware SHA is disabled and SHA1 is enabled in the active `sdkconfig`. |

## Project layout

- `main/app_main.cpp`: application startup and controller command handling.
- `main/MicroRosController/`: micro-ROS connection and message handling.
- `main/RobotController/`: robot control and kinematics.
- `main/PneumaticStepper/`: stepper and valve control.
- `main/InputController/`: input handling and hardware interfaces.
- `main/pinDefinitions.h`: hardware pin assignments.
- `components/micro_ros_espidf_component/`: bundled micro-ROS integration.
- `sdkconfig.defaults`: default ESP-IDF configuration.
- `platformio.ini`: PlatformIO board and environment configuration. (not used)

The `platformio.ini` serial ports are specific to the original development machine. This guide uses `idf.py` with an explicit port; adjust the PlatformIO ports separately if you use its upload tasks.
