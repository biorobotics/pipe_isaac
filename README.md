# pipe_issac

Isaac simulator for the Storm Runner Robot

## Dependencies

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Isaac Simulator 4.5](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)

## Install

1. Install packages:

```sh
sudo apt install -y libeigen3-dev libyaml-cpp-dev libgtest-dev python3-osrf-pycommon python3-colcon-common-extensions ros-humble-joy
```

2. Clone this repo to the `src` folder of your colcon ws:
```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone ADD LINK HERE
```

3. Install python packages

```sh
pip3 install -r requirements.txt
```

4. Install ROS dependencies

```sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y
```

5. To build the ROS packages, in the `colcon_ws`, run

```sh
colcon build
source install/setup.bash
```


## Usage

To launch the simulator:

```sh
# If downloaded Isaac sim, use default path: 
~/isaacsim/python.sh ~/colcon_ws/src/pipe_isaac/pipe_issac_sim/scripts/launch_sim.py

# Else, use path to Isaac sim installation
{ISSACSIM_PATH}/python.sh ~/colcon_ws/src/pipe_isaac/pipe_issac_sim/scripts/launch_sim.py 
```

ADD USAGE W/ ROS CONTROLLER