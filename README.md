# MRS visitor

Go to `mrs_msgs/Reference` while avoiding obstacles and staying within world bounds.

## How to use:

To launch the node:

```bash
roslaunch mrs_visitor mrs_visitor.launch
```

To specify custom world bounds or obstacles files use:

```bash
roslaunch mrs_visitor mrs_visitor.launch world_bounds_file:=/path/to/world_bounds.yml obstacles_file:=/path/to/obstacles.[dae|asc]
```

`world_bounds_file` is a YAML file that specifies the world bounds. See [example](resources/world_bounds.yaml).

`obstacles_file` can either be:

- Collada `dae` file ([example](resources/models/Towers/meshes/towers.dae))
- `.asc` file that is just a list of points in the format `x y z` ([example](resources/obstacles.asc))

## Whole example

```bash
$(rospack find mrs_visitor)/tmux_scripts/start.sh
```

This will launch custom gazebo world, spawn a UAV and command it to inspect a point.
