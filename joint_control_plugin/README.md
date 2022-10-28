## Write A Joint Control Plugin
In this tutorial we will understand how to make a model plugin to control gazebo joints.


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

**Launch gazebo simulator with joint control plugin**
```
roslaunch joint_control_plugin joint.launch 
```



[Joint Control Tutorial Document](https://sites.google.com/view/gazebo-plugin-tutorials/9-gazebo-joint-move-plugin)
