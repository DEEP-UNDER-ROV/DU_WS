# rov_supri

Packages to simulate and interface hardware--software to SUPRI ROV. This is an interface layer that connects the software system to the hardware system. This layer can be customized based on the hardware, while the upper layers like perception, planning, or control act as generic packages for arbitrary hardware.

## rov_mavros

Package to connect the upper layer with mavros. Currently it provides a bridge from slam to mavros and a launch file to launch mavros easily. This package can be used for both sim and real deployment.

Below are the available launches.

### rov_mavros.launch.xml

Runs mavros and the slam to mavros node.

<div align="center">

| argument | default | description |
|:---:|:---:|:---:|
| use_sim | false | set to true if using sim |
| param_dir | rov_mavros/config | directory path to config |
| pluginlists_file | apm_pluginlists.yaml | mavros plugin list filename |
| apm_config_file | apm_config.yaml | mavros apm config filename |
| mavros_param_file | mavros.yaml | mavros param filename |
| slam_mavros_file | slam_to_mavros.yaml | slam to mavros param filename |
| sim_url | udp://:14550@ | fcu url used in sim |
| real_fc_url | serial:///dev/ttyACM0:921600 | fcu url used in real deployment |

</div>

The launch file also calls:

### slam_to_mavros.launch.xml

Runs the slam to mavros node, which transforms and sends slam pose and velocity to the mavros vision plugin.

<div align="center">

| argument | default | description |
|:---:|:---:|:---:|
| use_sim | false | set to true if using sim |
| param_dir | rov_mavros/config | directory path to config |
| param_file | slam_to_mavros.yaml | slam to mavros param filename |

</div>

## rov_sim

This package contains everything to run the rov simulation, like models, params, and worlds. This package needs ardupilot_gazebo and Ardusupri to run.

Below are the available launches.

### rov_sim.launch.xml

Runs ArduSub SITL, gazebo with ros_gz_bridge, and D455 static tf.

<div align="center">

| argument | default | description |
|:---:|:---:|:---:|
| param_dir | rov_sim/config | directory path to config |
| param_file | ping.params | extra ArduSub param filename |
| bridge_file | rov_bridge.yaml | ros_gz_bridge config filename |
| world | shipwreck.world | world filename inside rov_sim/worlds |
| use_bridge | true | run ros_gz_bridge and image relays |
| instance | 0 | ArduSub instance id |
| sysid | 1 | MAVLink system id |
| use_map | false | open MAVProxy map |
| use_console | false | open MAVProxy console |
| use_tf | true | publish D455 static tf |

</div>

The launch file also calls:

### sitl.launch.xml

Runs ArduSub SITL from Ardusupri with the vectored frame and JSON model.

<div align="center">

| argument | default | description |
|:---:|:---:|:---:|
| param_dir | rov_sim/config | directory path to config |
| param_file | ardusupri.params | extra ArduSub param filename |
| instance | 0 | ArduSub instance id |
| sysid | 1 | MAVLink system id |
| use_map | false | open MAVProxy map |
| use_console | false | open MAVProxy console |

</div>

### gz.launch.xml

Runs gazebo with the selected world, ros_gz_bridge, and relays the infra images to `/corrected/{left,right}/image_raw`.

<div align="center">

| argument | default | description |
|:---:|:---:|:---:|
| param_dir | rov_sim/config | directory path to config |
| bridge_file | rov_bridge.yaml | ros_gz_bridge config filename |
| world | shipwreck.world | world filename inside rov_sim/worlds |
| use_bridge | true | run ros_gz_bridge and image relays |

</div>

### d455_tf.launch.xml

Publishes the static tf for the D455 camera.
