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
roslaunch subscriber_model_plugin gazebo_subscriber.launch
```


[Subscriber Tutorial Document](https://sites.google.com/view/gazebo-plugin-tutorials/4-gazebo-subscriber-plugin)