## Write A ROS Gazebo Light Control Plugin
In this tutorial, we will add ros subscriber to light plugin that we made in last tutorial. We are going to publish /light_color topic using terminal & gazebo plugin with subscribe to it & base on the data on ros topic gazebo plugin is going to publish on gazebo topic ~/light/modify which will change light color in gazebo simulator. 



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

**Launch gazebo simulator with simple ros light control plugin**
```
roslaunch light_control_ros simple_lightcontroller.launch
```

**Launch gazebo simulator with ros light control plugin***
```
roslaunch light_control_ros lightcontroller.launch
```

**Publish once on /light_color topic:** 
<br />

**red color**
```
rostopic pub -1 /light_color std_msgs/String "data: 'red'"
```
<br />

**green color**
```
rostopic pub -1 /light_color std_msgs/String "data: 'green'"
```
<br />

**blue color**
```
rostopic pub -1 /light_color std_msgs/String "data: 'blue'"
```

[ROS Gazebo Light Control Plugin Tutorial Document](https://sites.google.com/view/gazebo-plugin-tutorials/8-ros-gazebo-light-control-plugin)