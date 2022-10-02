## Write A ROS Gazebo Model Plugin
In this tutorial we will understand how to write a rso gazebo model plugin to push a model depending on the messageon the
ros topic. It has 2 methods to code ros subscriber in gazebo plugin. first one is simple & other one is bit advance using subscribersoptions which is mostly used in most of the ros gazebo plugins. 


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

**Launch gazebo simulator with simple ros model plugin**
```
roslaunch model_plugin_ros simple_rosplugin.launch
```

**Launch gazebo simulator with ros model plugin***
```
roslaunch model_plugin_ros rosplugin.launch
```

**Publish once on /model_move_up topic** 
```
rostopic pub -1 /model_move_up std_msgs/Bool "data: true"
```


[ROS Model Tutorial Document](https://sites.google.com/view/gazebo-plugin-tutorials/6-ros-gazebo-model-plugin)