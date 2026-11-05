# DU_WS

ROS 2 workspace for the Deep Under ROV.

Tested on: **Ubuntu 24.04 + ROS 2 Jazzy**.


## Installation

### 1. Clone

```bash
mkdir -p ~/DU_WS && cd ~/DU_WS
git clone --recursive https://github.com/DEEP-UNDER-ROV/DU_WS.git .
```

If you already cloned without `--recursive`:

```bash
cd ~/DU_WS
git submodule update --init --recursive
```

### 2. System dependencies

```bash
cd ~/DU_WS
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. ArduSupri SITL (simulation only)

Install prereqs

```bash
cd ~/DU_WS/src/rov_supri/thirdparty/Ardusupri
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

```

build the sub
```bash
./waf sub
```

### 4. Build

The first build compiles Pangolin and ORB-SLAM3 through `orb_slam3_vendor`, which takes a while and uses a lot of RAM:

```bash
cd ~/DU_WS
MAKEFLAGS=-j4 colcon build --symlink-install
source install/setup.zsh   # or setup.bash
```


## Running

All top-level launch files live in `rov_bringup`, with their parameters in `rov_bringup/config/`.

| Launch | Purpose |
|---|---|
| `rov.launch.xml` | Full ROV stack |
| `perception.launch.xml` | Perception pipeline |
| `slam.launch.xml` | ORB-SLAM3 |

```bash
ros2 launch rov_bringup rov.launch.xml
ros2 launch rov_bringup perception.launch.xml
ros2 launch rov_bringup slam.launch.xml
```

List the available arguments of any launch file with:

```bash
ros2 launch rov_bringup slam.launch.xml --show-args
```

### Configuration

Parameters are kept in YAML under `rov_bringup/config/`:

```
config/
├── mavros/       MAVROS and bridge parameters
├── perception/   SLAM, stereo depth, AprilTag parameters
└── sim/          simulation parameters
```

