# clearpath_simulator

## Setup

Prerequisites:
  - Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### Gazebo Harmonic

See [Gazebo Installation](https://gazebosim.org/docs/latest/ros_installation/) for more information
on installing Gazebo.

```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

### Workspace

```
mkdir ~/clearpath_ws/src -p
cd ~/clearpath_ws/src
git clone https://github.com/clearpathrobotics/clearpath_simulator.git
cd ~/clearpath_ws
rosdep install -r --from-paths src -i -y
colcon build --symlink-install
```

### Setup path

```
mkdir ~/clearpath
```

Copy your `robot.yaml` into `~/clearpath`

## Launch

```
ros2 launch clearpath_gz simulation.launch.py
```

## Worlds

The `clearpath_gz` package includes several simulation worlds. To select a specific world, use the
`world` launch parameter, e.g.
```
ros2 launch clearpath_gz simulation.launch.py world:=pipeline
```

Available worlds are:

| World                 | Description                                                                                                            | Screenshots                  | Geographic Location      |
|-----------------------|------------------------------------------------------------------------------------------------------------------------|------------------------------|--------------------------|
| `construction`        | The same floorplan as the `office` world, but under construction. Features non-solid walls and debris piles.           | [link](docs/construction.md) | Waterloo ON, Canada      |
| `office`              | The same floorplan as the `construction` world. Features narrow hallways, doorways, meeting rooms, and loading docks.  | [link](docs/office.md)       | Waterloo ON, Canada      |
| `orchard`             | An outdoor, agricultural environment featuring rows of trees. The terrain has small slopes, but is mostly flat.        | [link](docs/orchard.md)      | Nikea, Greece            |
| `pipeline`            | A rugged, outdoor environment featuring steeper hills, a river and bridge, a small cave, solar panels, and a pipeline. | [link](docs/pipeline.md)     | Northern Alberta, Canada |
| `solar_farm`          | An outdoor, agricultural environmentf featuring gentle hills, a barn, rows of solar panels, and fences.                | [link](docs/solar_farm.md)   | Stonewall MB, Canada     |
| `warehouse` (default) | A flat, indoor warehouse environment. Features shelves and people.                                                     | [link](docs/warehouse.md)    | Rio de Janeiro, Brazil   |


## Creating Map Tiles

The `orchard`, `pipeline`, and `solar_farm` worlds include geotagged TIF images in the `geotif`
directory. These images can be used to generate map tiles of the simulation environment, if
desired.

To generate the tiles, first install the `gdal-bin` package:
```bash
sudo apt install gdal-bin
```

Then run the following command to generate the tiles:
```bash
gdal2tiles.py $(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/geotif/WORLD_geo.tif
```
substituting `WORLD` with `orchard`, `pipeline`, or `solar_farm`.

The generated files will be located in the current working directory in a new directory called
`WORLD_geo` (e.g. `pipeline_geo`).

Note that while the simulation worlds' locations have been chosen to be geographically similar to
the envrionments depicted, the simulations are wholly fictional locations; the generated tiles
will not mesh seamlessly into any satellite map of the region depicted.