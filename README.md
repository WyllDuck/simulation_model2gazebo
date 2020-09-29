#### Fell free to improve the documentation on this project! [Need Help? Tips on how to modify README.md files here!](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)


# USEFULL TIPS:
### Close all the ROS & Gazebo processes:
```
killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
killall -9 rosmaster
killall -9 roscore
```

### Purge all python compile files:
```
find . -name '*.pyc' -delete
```

# FIRST STEPS TO MAKE YOUR _**GAZEBO_ROS**_ PROJECT: 
### Create a _**catkin workspace**_:
  * ``` mkdir -p ~/CATKIN_WORSPACE_NAME/src```
  * ``` cd ~/CATKIN_WORKSPACE_NAME/ ```
  * ``` catkin_make ```

### Once your _catkin workspace_ has been created navigate to the _**src**_ folder to create your first package.
### Create an empty package :
  * ``` catkin_create_pkg NAME_PROJECT ```
 

### Edit _**package.xml**_ and create you first _.launch_ file : 
 ![#f03c15](https://placehold.it/15/f03c15/000000?text=+) If you don't do this steps the _world models_ you create will be unaccessible from your **gazebo_ros** project
  * Add this this term in the _**package.xml**_ file inside the _tag_ **package**. This allows **Gazebo** to access the meshes saved on your package, inside the folder _meshes_. _It's preferably to add this lines at the end of the file_. 
  ``` xml
  <package>
  ...
    <export>
      <gazebo_ros gazebo_model_path="${prefix}/models"/>
      <gazebo_ros gazebo_media_path="${prefix}/models"/>
    </export>
  </package>
  ```
  * Use this _.launch_ file as a template for others: 

  ![#1589F0](https://placehold.it/15/1589F0/000000?text=+) This way every time you create a new _.launch_ file you won't forget to add this two *critical lines*

  ```xml
  <launch>
    ...
    <env name="GAZEBO_MODEL_PATH" value="$(find upc_driverless_gazebo)"/>
    <env name="GAZEBO_RESOURCE_PATH" value="$(find upc_driverless_gazebo)"/>
    ...
  </launch>
  ```


# USEFUL COMMAND FOR ROS:

### This command runs a ROS .launch file from the **_gazebo_ros_** package, additionaly **_roslaunch_** is passing arguments: _( The arguments can change form file to file )._
  * ``` roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true verbose:=true ```

# .launch FILES:

### This a simple .launch program that opens a .world file. World files define the spaces in which we simulate our robots. 
```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/mud.world"/> <!-- The world_name changes in function of which file you want to open -->
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="recording" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```
### We are importing _**empty_world.launch**_ because the package _**gazebo_ros**_ already defines the basic function of the world.
```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
```
### When it's time to pass the parameters to the _**empty_world.launch**_ file we change the _*world_name*_ argument to the one of the file we want to open. (Note: You must set the path to the file in relation to the directory you are working from, ROS has different tools to help you with this).
```xml
<arg name="world_name" value="worlds/mud.world"/> <!-- The world_name value changes in function of which file you want to open -->
```


# RECOMMENDATIONS:
### By adding this snippet at the beginning of your file the editor will automaticly set the type to _**XML**_ saving you time and making the code easier to read. 
```xml
<?xml version="1.0" ?>
```
### Caution with this: 
  * When writting code on any **XML** _(.world, .launch, .urdf, ...)_ file remember to name links with different names. For instances the **SDF** files, that are responsible for diffining the maps, were not responding accordingly to what was expected because all the tags _link_ had the same name. This meant that all the cones concured in the same position. The _**Visual Studio Code**_ marketapp _**Insert Numbers**_ help to speed up the process of name each link _tag_.

# SOURCES AND OTHER INFORMATION:
| Website        | Links           | Sujects converd  |
| :------------- |:-------------| :-------------|
| Gazebo Tutorials      | http://gazebosim.org/tutorials/?tut=ros_roslaunch | roslaunch, world files, urdf models |
| Gazebo Tutorials       | http://gazebosim.org/tutorials?tut=build_world&cat=build_world      |   edit world files,  save world files |
| Gazebo Tutorials    | http://gazebosim.org/tutorials/?tut=ros_urdf#Nextsteps      |   udrf models |
| Gazebo Tutorials    | http://gazebosim.org/tutorials?tut=ros_gzplugins      |    plugins|
| ROS Tutorials | http://wiki.ros.org/roslaunch/XML | launch files |
| ROS Tutorials | http://wiki.ros.org/urdf/XML/Transmission | urdf Transmisions |
| ROS Tutorials | http://wiki.ros.org/controller_manager | controller manager |

