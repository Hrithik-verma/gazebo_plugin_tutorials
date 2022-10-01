## Write A Gazebo World Plugin
In this tutorial we will understand how to write a gazebo world plugin


# Running Simulation <br />
**asssuption**: Its assume that your are within ros_workspacew & source the terminals. If not than please change directory to respective workspace & source it with follow command:
```
cd <workspace>
source devel/setup.bash
```

**In case you made any changes to plugin code, please do**
```
catkin_make
```

**Launch gazebo simulator with plugin**
```
roslaunch world_plugin gazebo_world_plugin.launch
```


[Tutorial Document](https://sites.google.com/view/gazebo-plugin-tutorials/3-write-a-world-plugin)