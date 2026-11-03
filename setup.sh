#!/bin/bash
set -eu

if [ "$EUID" -eq 0 ]; then
    echo "Do not run as root; the script calls sudo where needed." >&2
    exit 1
fi

git submodule update --init --recursive

sudo apt-get update
sudo apt-get install -y \
    python-is-python3 python3-dev python3-pip pipx python3-rosdep \
    libxml2-dev libxslt1-dev \
    libunwind-dev libgz-sim8-dev libgz-rendering8-dev \
    rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

set +u
source /opt/ros/jazzy/setup.bash
set -u

rosdep install --from-paths src --ignore-src -y

python3 -m pip install --user --break-system-packages \
    "future>=1.0.0" "empy==3.3.4" pexpect

pipx install MAVProxy || pipx upgrade MAVProxy
pipx ensurepath

python3 -c "import em, future; assert tuple(map(int, future.__version__.split('.')[:2])) >= (1, 0)"
python3 -c "import pexpect"

echo "Done. Open a new terminal so ~/.local/bin is on PATH, then run colcon build."
