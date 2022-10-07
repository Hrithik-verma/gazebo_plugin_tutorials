## Write A Gazebo Model Plugin
In this tutorial we will understand how to write a gazebo model plugin to push a model.


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

**Launch gazebo simulator with model plugin**
```
roslaunch model_plugin model.launch
```

**Launch gazebo simulator with custom model plugin**
```
roslaunch model_plugin custom_model.launch
```


[Model Tutorial Document](https://sites.google.com/view/gazebo-plugin-tutorials/5-write-a-model-plugin)
