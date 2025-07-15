# BME680 Pressure Sensor

bme680 is a ROS 2 interface for the BME680 pressure sensor.

## Installation

To install the project, first clone the repository to the `src/` directory of
your ROS 2 workspace

```bash
git clone git@github.com:Robotic-Decision-Making-Lab/bme680.git
```

Then install the project dependencies using rosdep

```bash
rosdep install --from paths src -y --ignore-src
```

Finally, build the workspace using colcon

```bash
colcon build && source install/setup.bash
```

## Usage

The BME680 ROS 2 driver can be launched with

```bash
ros2 launch bme680_driver bme680.launch.yaml
```

## Getting help

If you have questions regarding usage of bme680 or regarding contributing
to this project, please submit an issue to our [Issue Tracker](https://github.com/Robotic-Decision-Making-Lab/bme680/issues).

## License

bme680 is released under the MIT license.
