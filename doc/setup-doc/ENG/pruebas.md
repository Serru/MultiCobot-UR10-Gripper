# Performing tests on the finished installation

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/pruebas.md) | **English**

## If there are *warnings* during the build 

### Ignored warnings:

```bash
WARNING: Package 'ur_modern_driver' is deprecated (This package has been deprecated. Users of CB3 and e-Series controllers should migrate to ur_robot_driver.)

CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:418 (message):
  catkin_package() include dir
  '/home/miguel/MultiCobot-UR10-Gripper/build/gazebo-pkgs/gazebo_grasp_plugin/..'
  should be placed in the devel space instead of the build space
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_grasp_plugin/CMakeLists.txt:31 (catkin_package)
  
/home/miguel/MultiCobot-UR10-Gripper/src/ros_control/hardware_interface/include/hardware_interface/internal/interface_manager.h:69:85: warning: type qualifiers ignored on function return type [-Wignored-qualifiers]
   static const void callConcatManagers(typename std::vector<T*>& managers, T* result)
```
### Resolved warnings: 

**gazebo_version_helpers warning:**

```bash
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'gazebo' but neither 'gazebo_INCLUDE_DIRS' nor
  'gazebo_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_version_helpers/CMakeLists.txt:26 (catkin_package)
```

Modify the file `~/MultiCobot-UR10-Gripper/src/gazebo-pkgs/gazebo_version_helpers/CMakeLists.txt`, starting at line 26 as follows:

```bash
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_version_helpers
  CATKIN_DEPENDS gazebo_ros roscpp
  DEPENDS GAZEBO 
)
``` 

**gazebo_grasp_plugin warning:** 

```bash
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'gazebo' but neither 'gazebo_INCLUDE_DIRS' nor
  'gazebo_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_grasp_plugin/CMakeLists.txt:31 (catkin_package)
[...]
```

Modify the file `~/MultiCobot-UR10-Gripper/src/gazebo-pkgs/gazebo_grasp_plugin/CMakeLists.txt`, starting at line 31 as follows:

```bash
catkin_package(
  # Binary directory required for proto headers inclusion to work, because install commands don't
  # get executed in devel space. The directory above is required so that an include of
  # <gazebo_grasp_plugin/msgs/grasp_event.pb.h> 
  # also works in devel space like it needs to be in install space.
  # Probably we can find a better solution for this, but until then this
  # fix will be OK.
  INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}/..
  LIBRARIES gazebo_grasp_fix gazebo_grasp_msgs
  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp std_msgs gazebo_version_helpers
  DEPENDS GAZEBO
)
``` 

**gazebo_grasp_plugin_ros warning:**

```bash
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'gazebo' but neither 'gazebo_INCLUDE_DIRS' nor
  'gazebo_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_grasp_plugin_ros/CMakeLists.txt:34 (catkin_package)
[...]
```

Modify the file `~/MultiCobot-UR10-Gripper/src/gazebo-pkgs/gazebo_grasp_plugin_ros/CMakeLists.txt`, starting at line 34 as follows:

```bash
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_grasp_plugin_ros
 CATKIN_DEPENDS gazebo_grasp_plugin message_runtime roscpp
 DEPENDS GAZEBO
)
``` 

## Small check of the system installation 

### Activation of the current working environment

```bash
cd ~/MultiCobot-UR10-Gripper
source ~/MultiCobot-UR10-Gripper/devel/setup.bash
``` 

### Testing the basic configuration
After you install and configure the system, you can run tests to verify that the system works before making any further changes. It is not specified which tests can be run, as the source code repositories contain instructions for running small demos that are very useful for understanding what they can do. 

The working environment should look like the following after installing all the repositories:

```bash
miguel@Omen:~/MultiCobot-UR10-Gripper/src$ ls
CMakeLists.txt        geometry                  ros_control
gazebo-pkgs           leap_motion               roslint
gazebo_ros_pkgs       object_recognition_msgs   universal_robot
general-message-pkgs  robotiq_2finger_grippers  ur_modern_driver
``` 

<details>
	<summary> Structure and detailed content of the directories </summary>

```text
src/
├── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
├── gazebo-pkgs
│   ├── Dockerfile
│   ├── gazebo_grasp_plugin
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── gazebo_grasp_plugin
│   │   │       ├── GazeboGraspFix.h
│   │   │       └── GazeboGraspGripper.h
│   │   ├── msgs
│   │   │   ├── CMakeLists.txt
│   │   │   └── grasp_event.proto
│   │   ├── package.xml
│   │   └── src
│   │       ├── GazeboGraspFix.cpp
│   │       └── GazeboGraspGripper.cpp
│   ├── gazebo_grasp_plugin_ros
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   └── GazeboGraspEvent.msg
│   │   ├── package.xml
│   │   ├── README.md
│   │   └── src
│   │       └── grasp_event_republisher.cpp
│   ├── gazebo_state_plugins
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── GazeboMapPublisher.yaml
│   │   │   ├── GazeboObjectInfo.yaml
│   │   │   └── WorldPlugins.yaml
│   │   ├── include
│   │   │   └── gazebo_state_plugins
│   │   │       ├── GazeboMapPublisher.h
│   │   │       └── GazeboObjectInfo.h
│   │   ├── launch
│   │   │   └── plugin_loader.launch
│   │   ├── package.xml
│   │   ├── src
│   │   │   ├── GazeboMapPublisher.cpp
│   │   │   └── GazeboObjectInfo.cpp
│   │   └── test
│   │       └── object_info_request.cpp
│   ├── gazebo_test_tools
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── FakeObjectRecognizer.yaml
│   │   │   └── ObjectTFBroadcaster.yaml
│   │   ├── include
│   │   │   └── gazebo_test_tools
│   │   │       ├── FakeObjectRecognizer.h
│   │   │       └── gazebo_cube_spawner.h
│   │   ├── launch
│   │   │   ├── fake_object_recognizer.launch
│   │   │   ├── gazebo_fake_object_recognition.launch
│   │   │   ├── object_tf_broadcaster.launch
│   │   │   └── spawn_and_recognize_cube.launch
│   │   ├── package.xml
│   │   ├── src
│   │   │   ├── cube_spawner.cpp
│   │   │   ├── cube_spawner_node.cpp
│   │   │   ├── FakeObjectRecognizer.cpp
│   │   │   ├── fake_object_recognizer_node.cpp
│   │   │   └── SetGazeboPhysicsClient.cpp
│   │   ├── srv
│   │   │   └── RecognizeGazeboObject.srv
│   │   └── test
│   │       └── fake_object_recognizer_cmd.cpp
│   ├── gazebo_version_helpers
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── gazebo_version_helpers
│   │   │       └── GazeboVersionHelpers.h
│   │   ├── package.xml
│   │   └── src
│   │       └── GazeboVersionHelpers.cpp
│   ├── gazebo_world_plugin_loader
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── WorldPluginsTemplate.config
│   │   ├── include
│   │   │   └── gazebo_world_plugin_loader
│   │   │       └── GazeboPluginLoader.h
│   │   ├── launch
│   │   │   └── plugin_loader_template.launch
│   │   ├── package.xml
│   │   └── src
│   │       └── GazeboPluginLoader.cpp
│   ├── LICENSE
│   ├── README.md
│   └── TODO.md
├── gazebo_ros_pkgs
│   ├── CONTRIBUTING.md
│   ├── gazebo_dev
│   │   ├── CHANGELOG.rst
│   │   ├── cmake
│   │   │   └── gazebo_dev-extras.cmake
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── gazebo_msgs
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── ContactsState.msg
│   │   │   ├── ContactState.msg
│   │   │   ├── LinkState.msg
│   │   │   ├── LinkStates.msg
│   │   │   ├── ModelState.msg
│   │   │   ├── ModelStates.msg
│   │   │   ├── ODEJointProperties.msg
│   │   │   ├── ODEPhysics.msg
│   │   │   ├── PerformanceMetrics.msg
│   │   │   ├── SensorPerformanceMetric.msg
│   │   │   └── WorldState.msg
│   │   ├── package.xml
│   │   └── srv
│   │       ├── ApplyBodyWrench.srv
│   │       ├── ApplyJointEffort.srv
│   │       ├── BodyRequest.srv
│   │       ├── DeleteLight.srv
│   │       ├── DeleteModel.srv
│   │       ├── GetJointProperties.srv
│   │       ├── GetLightProperties.srv
│   │       ├── GetLinkProperties.srv
│   │       ├── GetLinkState.srv
│   │       ├── GetModelProperties.srv
│   │       ├── GetModelState.srv
│   │       ├── GetPhysicsProperties.srv
│   │       ├── GetWorldProperties.srv
│   │       ├── JointRequest.srv
│   │       ├── SetJointProperties.srv
│   │       ├── SetJointTrajectory.srv
│   │       ├── SetLightProperties.srv
│   │       ├── SetLinkProperties.srv
│   │       ├── SetLinkState.srv
│   │       ├── SetModelConfiguration.srv
│   │       ├── SetModelState.srv
│   │       ├── SetPhysicsProperties.srv
│   │       └── SpawnModel.srv
│   ├── gazebo_plugins
│   │   ├── cfg
│   │   │   ├── CameraSynchronizer.cfg
│   │   │   ├── GazeboRosCamera.cfg
│   │   │   ├── GazeboRosOpenniKinect.cfg
│   │   │   └── Hokuyo.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── gazebo_plugins
│   │   │       ├── gazebo_ros_block_laser.h
│   │   │       ├── gazebo_ros_bumper.h
│   │   │       ├── gazebo_ros_camera.h
│   │   │       ├── gazebo_ros_camera_utils.h
│   │   │       ├── gazebo_ros_depth_camera.h
│   │   │       ├── gazebo_ros_diff_drive.h
│   │   │       ├── gazebo_ros_elevator.h
│   │   │       ├── gazebo_ros_f3d.h
│   │   │       ├── gazebo_ros_force.h
│   │   │       ├── gazebo_ros_ft_sensor.h
│   │   │       ├── gazebo_ros_gpu_laser.h
│   │   │       ├── gazebo_ros_hand_of_god.h
│   │   │       ├── gazebo_ros_harness.h
│   │   │       ├── gazebo_ros_imu.h
│   │   │       ├── gazebo_ros_imu_sensor.h
│   │   │       ├── gazebo_ros_joint_pose_trajectory.h
│   │   │       ├── gazebo_ros_joint_state_publisher.h
│   │   │       ├── gazebo_ros_joint_trajectory.h
│   │   │       ├── gazebo_ros_laser.h
│   │   │       ├── gazebo_ros_moveit_planning_scene.h
│   │   │       ├── gazebo_ros_multicamera.h
│   │   │       ├── gazebo_ros_openni_kinect.h
│   │   │       ├── gazebo_ros_p3d.h
│   │   │       ├── gazebo_ros_planar_move.h
│   │   │       ├── gazebo_ros_projector.h
│   │   │       ├── gazebo_ros_prosilica.h
│   │   │       ├── gazebo_ros_range.h
│   │   │       ├── gazebo_ros_skid_steer_drive.h
│   │   │       ├── gazebo_ros_template.h
│   │   │       ├── gazebo_ros_tricycle_drive.h
│   │   │       ├── gazebo_ros_triggered_camera.h
│   │   │       ├── gazebo_ros_triggered_multicamera.h
│   │   │       ├── gazebo_ros_utils.h
│   │   │       ├── gazebo_ros_vacuum_gripper.h
│   │   │       ├── gazebo_ros_video.h
│   │   │       ├── MultiCameraPlugin.h
│   │   │       ├── PubQueue.h
│   │   │       └── vision_reconfigure.h
│   │   ├── Media
│   │   │   └── models
│   │   │       └── chair
│   │   │           ├── doc.kml
│   │   │           ├── images
│   │   │           │   ├── texture0.jpg
│   │   │           │   └── texture1.jpg
│   │   │           ├── models
│   │   │           │   ├── Chair.dae
│   │   │           │   └── Chair.stl
│   │   │           └── textures.txt
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   ├── gazebo_model
│   │   │   ├── set_pose.py
│   │   │   ├── set_wrench.py
│   │   │   └── test_range.py
│   │   ├── setup.py
│   │   ├── src
│   │   │   ├── camera_synchronizer.cpp
│   │   │   ├── gazebo_plugins
│   │   │   │   ├── gazebo_plugins_interface.py
│   │   │   │   └── __init__.py
│   │   │   ├── gazebo_ros_block_laser.cpp
│   │   │   ├── gazebo_ros_bumper.cpp
│   │   │   ├── gazebo_ros_camera.cpp
│   │   │   ├── gazebo_ros_camera_utils.cpp
│   │   │   ├── gazebo_ros_depth_camera.cpp
│   │   │   ├── gazebo_ros_diff_drive.cpp
│   │   │   ├── gazebo_ros_elevator.cpp
│   │   │   ├── gazebo_ros_f3d.cpp
│   │   │   ├── gazebo_ros_force.cpp
│   │   │   ├── gazebo_ros_ft_sensor.cpp
│   │   │   ├── gazebo_ros_gpu_laser.cpp
│   │   │   ├── gazebo_ros_hand_of_god.cpp
│   │   │   ├── gazebo_ros_harness.cpp
│   │   │   ├── gazebo_ros_imu.cpp
│   │   │   ├── gazebo_ros_imu_sensor.cpp
│   │   │   ├── gazebo_ros_joint_pose_trajectory.cpp
│   │   │   ├── gazebo_ros_joint_state_publisher.cpp
│   │   │   ├── gazebo_ros_joint_trajectory.cpp
│   │   │   ├── gazebo_ros_laser.cpp
│   │   │   ├── gazebo_ros_moveit_planning_scene.cpp
│   │   │   ├── gazebo_ros_multicamera.cpp
│   │   │   ├── gazebo_ros_openni_kinect.cpp
│   │   │   ├── gazebo_ros_p3d.cpp
│   │   │   ├── gazebo_ros_planar_move.cpp
│   │   │   ├── gazebo_ros_projector.cpp
│   │   │   ├── gazebo_ros_prosilica.cpp
│   │   │   ├── gazebo_ros_range.cpp
│   │   │   ├── gazebo_ros_skid_steer_drive.cpp
│   │   │   ├── gazebo_ros_template.cpp
│   │   │   ├── gazebo_ros_tricycle_drive.cpp
│   │   │   ├── gazebo_ros_triggered_camera.cpp
│   │   │   ├── gazebo_ros_triggered_multicamera.cpp
│   │   │   ├── gazebo_ros_utils.cpp
│   │   │   ├── gazebo_ros_vacuum_gripper.cpp
│   │   │   ├── gazebo_ros_video.cpp
│   │   │   ├── hokuyo_node.cpp
│   │   │   ├── MultiCameraPlugin.cpp
│   │   │   └── vision_reconfigure.cpp
│   │   ├── test
│   │   │   ├── bumper_test
│   │   │   │   ├── gazebo_ros_bumper.world
│   │   │   │   ├── test_bumper.launch
│   │   │   │   └── test_bumper.py
│   │   │   ├── camera
│   │   │   │   ├── camera16bit.cpp
│   │   │   │   ├── camera16bit.test
│   │   │   │   ├── camera16bit.world
│   │   │   │   ├── camera.cpp
│   │   │   │   ├── camera.h
│   │   │   │   ├── camera.test
│   │   │   │   ├── camera.world
│   │   │   │   ├── depth_camera.cpp
│   │   │   │   ├── depth_camera.test
│   │   │   │   ├── depth_camera.world
│   │   │   │   ├── distortion_barrel.cpp
│   │   │   │   ├── distortion_barrel.test
│   │   │   │   ├── distortion_barrel.world
│   │   │   │   ├── distortion.h
│   │   │   │   ├── distortion_pincushion.cpp
│   │   │   │   ├── distortion_pincushion.test
│   │   │   │   ├── distortion_pincushion.world
│   │   │   │   ├── multicamera.cpp
│   │   │   │   ├── multicamera.test
│   │   │   │   ├── multicamera.world
│   │   │   │   ├── triggered_camera.cpp
│   │   │   │   ├── triggered_camera.test
│   │   │   │   └── triggered_camera.world
│   │   │   ├── config
│   │   │   │   └── example_models.yaml
│   │   │   ├── multi_robot_scenario
│   │   │   │   ├── launch
│   │   │   │   │   ├── multi_robot_scenario.launch
│   │   │   │   │   ├── pioneer3dx.gazebo.launch
│   │   │   │   │   ├── pioneer3dx.rviz
│   │   │   │   │   └── pioneer3dx.urdf.launch
│   │   │   │   ├── meshes
│   │   │   │   │   ├── laser
│   │   │   │   │   │   └── hokuyo.dae
│   │   │   │   │   └── p3dx
│   │   │   │   │       ├── back_rim.stl
│   │   │   │   │       ├── back_sonar.stl
│   │   │   │   │       ├── center_hubcap.stl
│   │   │   │   │       ├── center_wheel.stl
│   │   │   │   │       ├── chassis.stl
│   │   │   │   │       ├── Coordinates
│   │   │   │   │       ├── front_rim.stl
│   │   │   │   │       ├── front_sonar.stl
│   │   │   │   │       ├── left_hubcap.stl
│   │   │   │   │       ├── left_wheel.stl
│   │   │   │   │       ├── right_hubcap.stl
│   │   │   │   │       ├── right_wheel.stl
│   │   │   │   │       ├── swivel.stl
│   │   │   │   │       └── top.stl
│   │   │   │   └── xacro
│   │   │   │       ├── camera
│   │   │   │       │   └── camera.xacro
│   │   │   │       ├── laser
│   │   │   │       │   ├── hokuyo_gpu.xacro
│   │   │   │       │   └── hokuyo.xacro
│   │   │   │       ├── materials.xacro
│   │   │   │       └── p3dx
│   │   │   │           ├── battery_block.xacro
│   │   │   │           ├── inertia_tensors.xacro
│   │   │   │           ├── pioneer3dx_body.xacro
│   │   │   │           ├── pioneer3dx_chassis.xacro
│   │   │   │           ├── pioneer3dx_plugins.xacro
│   │   │   │           ├── pioneer3dx_sonar.xacro
│   │   │   │           ├── pioneer3dx_swivel.xacro
│   │   │   │           ├── pioneer3dx_wheel.xacro
│   │   │   │           └── pioneer3dx.xacro
│   │   │   ├── p3d_test
│   │   │   │   ├── test_3_double_pendulums.launch
│   │   │   │   ├── test_3_single_pendulums.launch
│   │   │   │   ├── test_double_pendulum.launch
│   │   │   │   ├── test_link_pose.py
│   │   │   │   ├── test_single_pendulum.launch
│   │   │   │   └── worlds
│   │   │   │       ├── 3_double_pendulums.world
│   │   │   │       ├── 3_single_pendulums.world
│   │   │   │       ├── double_pendulum.world
│   │   │   │       └── single_pendulum.world
│   │   │   ├── pub_joint_trajectory_test.cpp
│   │   │   ├── range
│   │   │   │   └── range_plugin.test
│   │   │   ├── set_model_state_test
│   │   │   │   ├── set_model_state_test.cpp
│   │   │   │   ├── set_model_state_test_p2dx.world
│   │   │   │   └── set_model_state_test.test
│   │   │   ├── spawn_test
│   │   │   │   ├── parameter_server_test.launch
│   │   │   │   └── spawn_robots.sh
│   │   │   ├── test_worlds
│   │   │   │   ├── bumper_test.world
│   │   │   │   ├── elevator.world
│   │   │   │   ├── gazebo_ros_block_laser.world
│   │   │   │   ├── gazebo_ros_camera.world
│   │   │   │   ├── gazebo_ros_depth_camera.world
│   │   │   │   ├── gazebo_ros_gpu_laser.world
│   │   │   │   ├── gazebo_ros_laser.world
│   │   │   │   ├── gazebo_ros_range.world
│   │   │   │   ├── gazebo_ros_trimesh_collision.world
│   │   │   │   └── test_lasers.world
│   │   │   └── tricycle_drive
│   │   │       ├── launch
│   │   │       │   ├── tricycle_drive_scenario.launch
│   │   │       │   ├── tricycle.gazebo.launch
│   │   │       │   ├── tricycle.rviz
│   │   │       │   ├── tricycle_rviz.launch
│   │   │       │   └── tricycle.urdf.launch
│   │   │       └── xacro
│   │   │           ├── materials.xacro
│   │   │           └── tricycle
│   │   │               ├── inertia_tensors.xacro
│   │   │               ├── tricycle_body.xacro
│   │   │               ├── tricycle_chassis.xacro
│   │   │               ├── tricycle_plugins.xacro
│   │   │               ├── tricycle.xacro
│   │   │               ├── wheel_actuated.xacro
│   │   │               └── wheel.xacro
│   │   └── test2
│   │       ├── CMakeLists_tests_pkg.txt
│   │       ├── contact_tolerance
│   │       │   ├── contact_tolerance.cpp
│   │       │   └── contact_tolerance.launch
│   │       ├── large_models
│   │       │   ├── large_model.launch
│   │       │   ├── large_models.world
│   │       │   ├── large_model.urdf.xacro
│   │       │   ├── smaller_large_model.launch
│   │       │   └── smaller_large_model.urdf.xacro
│   │       ├── lcp_tests
│   │       │   ├── balance.launch
│   │       │   ├── balance.world
│   │       │   ├── stack.launch
│   │       │   ├── stacks.launch
│   │       │   ├── stacks.world
│   │       │   └── stack.world
│   │       ├── meshes
│   │       │   ├── cube_20k.stl
│   │       │   ├── cube_30k.stl
│   │       │   └── cube.wings
│   │       ├── spawn_model
│   │       │   ├── check_model.cpp
│   │       │   ├── spawn_box.cpp
│   │       │   ├── spawn_box_file.launch
│   │       │   ├── spawn_box.launch
│   │       │   └── spawn_box_param.launch
│   │       ├── trimesh_tests
│   │       │   └── test_trimesh.launch
│   │       ├── urdf
│   │       │   ├── box.urdf
│   │       │   └── cube.urdf
│   │       └── worlds
│   │           └── empty.world
│   ├── gazebo_ros
│   │   ├── cfg
│   │   │   └── Physics.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── gazebo_ros
│   │   │       └── gazebo_ros_api_plugin.h
│   │   ├── launch
│   │   │   ├── elevator_world.launch
│   │   │   ├── empty_world.launch
│   │   │   ├── mud_world.launch
│   │   │   ├── range_world.launch
│   │   │   ├── rubble_world.launch
│   │   │   ├── shapes_world.launch
│   │   │   └── willowgarage_world.launch
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   ├── debug
│   │   │   ├── gazebo
│   │   │   ├── gdbrun
│   │   │   ├── gzclient
│   │   │   ├── gzserver
│   │   │   ├── libcommon.sh
│   │   │   ├── perf
│   │   │   └── spawn_model
│   │   ├── setup.py
│   │   └── src
│   │       ├── gazebo_ros
│   │       │   ├── gazebo_interface.py
│   │       │   ├── gazebo_interface.pyc
│   │       │   └── __init__.py
│   │       ├── gazebo_ros_api_plugin.cpp
│   │       └── gazebo_ros_paths_plugin.cpp
│   ├── gazebo_ros_control
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── gazebo_ros_control
│   │   │       ├── default_robot_hw_sim.h
│   │   │       ├── gazebo_ros_control_plugin.h
│   │   │       └── robot_hw_sim.h
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── robot_hw_sim_plugins.xml
│   │   └── src
│   │       ├── default_robot_hw_sim.cpp
│   │       └── gazebo_ros_control_plugin.cpp
│   ├── gazebo_ros_pkgs
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── documentation
│   │   │   ├── gazebo_ros_api.odg
│   │   │   ├── gazebo_ros_api.pdf
│   │   │   ├── gazebo_ros_api.png
│   │   │   ├── gazebo_ros_transmission.odg
│   │   │   ├── gazebo_ros_transmission.pdf
│   │   │   └── gazebo_ros_transmission.png
│   │   └── package.xml
│   ├── README.md
│   └── SENSORS.md
├── general-message-pkgs
│   ├── Dockerfile
│   ├── LICENSE
│   ├── object_msgs
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── Object.msg
│   │   │   └── ObjectPose.msg
│   │   ├── package.xml
│   │   └── srv
│   │       ├── ObjectInfo.srv
│   │       └── RegisterObject.srv
│   ├── object_msgs_tools
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── ObjectTFBroadcaster.yaml
│   │   ├── include
│   │   │   └── object_msgs_tools
│   │   │       ├── ObjectFunctions.h
│   │   │       └── ObjectTFBroadcaster.h
│   │   ├── launch
│   │   │   └── object_tf_broadcaster.launch
│   │   ├── package.xml
│   │   └── src
│   │       ├── ObjectFunctions.cpp
│   │       ├── ObjectTFBroadcaster.cpp
│   │       ├── object_tf_broadcaster_node.cpp
│   │       └── register_object_client.cpp
│   ├── path_navigation_msgs
│   │   ├── action
│   │   │   ├── PathExecution.action
│   │   │   └── TransformPathExecution.action
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── README.md
├── geometry
│   ├── eigen_conversions
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── eigen_conversions
│   │   │       ├── eigen_kdl.h
│   │   │       └── eigen_msg.h
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   └── src
│   │       ├── eigen_kdl.cpp
│   │       └── eigen_msg.cpp
│   ├── geometry
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── kdl_conversions
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── kdl_conversions
│   │   │       └── kdl_msg.h
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   └── src
│   │       └── kdl_msg.cpp
│   ├── tf
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── conf.py
│   │   ├── doc
│   │   │   ├── bifrucation.gv
│   │   │   └── bifrucation.pdf
│   │   ├── include
│   │   │   └── tf
│   │   │       ├── exceptions.h
│   │   │       ├── LinearMath
│   │   │       │   ├── Matrix3x3.h
│   │   │       │   ├── MinMax.h
│   │   │       │   ├── QuadWord.h
│   │   │       │   ├── Quaternion.h
│   │   │       │   ├── Scalar.h
│   │   │       │   ├── Transform.h
│   │   │       │   └── Vector3.h
│   │   │       ├── message_filter.h
│   │   │       ├── tf.h
│   │   │       ├── time_cache.h
│   │   │       ├── transform_broadcaster.h
│   │   │       ├── transform_datatypes.h
│   │   │       └── transform_listener.h
│   │   ├── index.rst
│   │   ├── mainpage.dox
│   │   ├── msg
│   │   │   └── tfMessage.msg
│   │   ├── package.xml
│   │   ├── remap_tf.launch
│   │   ├── rosdoc.yaml
│   │   ├── scripts
│   │   │   ├── bullet_migration_sed.py
│   │   │   ├── groovy_compatibility
│   │   │   │   ├── tf_remap
│   │   │   │   └── view_frames
│   │   │   ├── python_benchmark.py
│   │   │   ├── tf_remap
│   │   │   └── view_frames
│   │   ├── setup.py
│   │   ├── src
│   │   │   ├── cache.cpp
│   │   │   ├── change_notifier.cpp
│   │   │   ├── empty_listener.cpp
│   │   │   ├── static_transform_publisher.cpp
│   │   │   ├── tf
│   │   │   │   ├── broadcaster.py
│   │   │   │   ├── broadcaster.pyc
│   │   │   │   ├── __init__.py
│   │   │   │   ├── listener.py
│   │   │   │   ├── listener.pyc
│   │   │   │   ├── tfwtf.py
│   │   │   │   ├── transformations.py
│   │   │   │   └── transformations.pyc
│   │   │   ├── tf.cpp
│   │   │   ├── tf_echo.cpp
│   │   │   ├── tf_monitor.cpp
│   │   │   ├── transform_broadcaster.cpp
│   │   │   └── transform_listener.cpp
│   │   ├── srv
│   │   │   └── FrameGraph.srv
│   │   ├── test
│   │   │   ├── cache_unittest.cpp
│   │   │   ├── method_test.py
│   │   │   ├── operator_overload.cpp
│   │   │   ├── python_debug_test.py
│   │   │   ├── quaternion.cpp
│   │   │   ├── speed_test.cpp
│   │   │   ├── testBroadcaster.cpp
│   │   │   ├── test_broadcaster.launch
│   │   │   ├── test_datatype_conversion.py
│   │   │   ├── testListener.cpp
│   │   │   ├── test_message_filter.cpp
│   │   │   ├── test_message_filter.xml
│   │   │   ├── testPython.py
│   │   │   ├── test_transform_datatypes.cpp
│   │   │   ├── tf_benchmark.cpp
│   │   │   ├── tf_unittest.cpp
│   │   │   ├── tf_unittest_future.cpp
│   │   │   ├── transform_listener_unittest.cpp
│   │   │   ├── transform_listener_unittest.launch
│   │   │   ├── transform_twist_test.cpp
│   │   │   ├── transform_twist_test.launch
│   │   │   └── velocity_test.cpp
│   │   ├── tf_python.rst
│   │   └── transformations.rst
│   └── tf_conversions
│       ├── CHANGELOG.rst
│       ├── CMakeLists.txt
│       ├── conf.py
│       ├── include
│       │   └── tf_conversions
│       │       ├── mainpage.dox
│       │       ├── tf_eigen.h
│       │       └── tf_kdl.h
│       ├── index.rst
│       ├── package.xml
│       ├── rosdoc.yaml
│       ├── setup.py
│       ├── src
│       │   ├── tf_conversions
│       │   │   ├── __init__.py
│       │   │   └── posemath.py
│       │   ├── tf_eigen.cpp
│       │   └── tf_kdl.cpp
│       └── test
│           ├── posemath.py
│           ├── test_eigen_tf.cpp
│           └── test_kdl_tf.cpp
├── leap_motion
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── camera_info
│   │   │   ├── leap_cal_left.yml
│   │   │   └── leap_cal_right.yml
│   │   ├── filter_params.yaml
│   │   ├── listener_params.yaml
│   │   └── RViz
│   │       ├── leap_camera.rviz
│   │       ├── leap_demo.rviz
│   │       └── leap_visualization.rviz
│   ├── inc
│   │   ├── lmc_filter_node.h
│   │   └── lmc_listener.h
│   ├── launch
│   │   ├── camera.launch
│   │   ├── demo.launch
│   │   ├── leap_camera.launch
│   │   ├── leap_stereo.launch
│   │   ├── sensor_sender.launch
│   │   └── visualization.launch
│   ├── LeapSDK
│   │   ├── include
│   │   │   ├── Leap.h
│   │   │   ├── Leap.i
│   │   │   └── LeapMath.h
│   │   ├── lib
│   │   │   ├── Leap.py
│   │   │   ├── x64
│   │   │   │   ├── LeapPython.so
│   │   │   │   └── libLeap.so
│   │   │   └── x86
│   │   │       ├── LeapPython.so
│   │   │       └── libLeap.so
│   │   └── util
│   │       ├── LeapScene.cpp
│   │       ├── LeapScene.h
│   │       ├── LeapUtil.cpp
│   │       ├── LeapUtilGL.cpp
│   │       ├── LeapUtilGL.h
│   │       └── LeapUtil.h
│   ├── msg
│   │   ├── Arm.msg
│   │   ├── Bone.msg
│   │   ├── Finger.msg
│   │   ├── Gesture.msg
│   │   ├── Hand.msg
│   │   ├── Human.msg
│   │   ├── leapcobotleft.msg
│   │   ├── leapcobotright.msg
│   │   ├── leap.msg
│   │   └── leapros.msg
│   ├── package.xml
│   ├── README.md
│   ├── scripts
│   │   ├── leap_interface.py
│   │   ├── sender.py
│   │   ├── skeleton_sender.py
│   │   └── subscriber.py
│   └── src
│       ├── leap_camera.cpp
│       ├── leap_hands.cpp
│       ├── lmc_camera_node.cpp
│       ├── lmc_driver_node.cpp
│       ├── lmc_filter_node.cpp
│       ├── lmc_listener.cpp
│       └── lmc_visualizer_node.cpp
├── object_recognition_msgs
│   ├── action
│   │   └── ObjectRecognition.action
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── msg
│   │   ├── ObjectInformation.msg
│   │   ├── ObjectType.msg
│   │   ├── RecognizedObjectArray.msg
│   │   ├── RecognizedObject.msg
│   │   ├── TableArray.msg
│   │   └── Table.msg
│   ├── package.xml
│   └── srv
│       └── GetObjectInformation.srv
├── robotiq_2finger_grippers
│   ├── README.md
│   ├── robotiq_2f_140_gripper_visualization
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── test_2f_140_model.launch
│   │   ├── meshes
│   │   │   ├── collision
│   │   │   │   ├── robotiq_arg2f_140_inner_finger.stl
│   │   │   │   ├── robotiq_arg2f_140_inner_knuckle.stl
│   │   │   │   ├── robotiq_arg2f_140_outer_finger.stl
│   │   │   │   ├── robotiq_arg2f_140_outer_knuckle.stl
│   │   │   │   ├── robotiq_arg2f_base_link.stl
│   │   │   │   └── robotiq_arg2f_coupling.stl
│   │   │   └── visual
│   │   │       ├── robotiq_arg2f_140_inner_finger.stl
│   │   │       ├── robotiq_arg2f_140_inner_knuckle.stl
│   │   │       ├── robotiq_arg2f_140_outer_finger.stl
│   │   │       ├── robotiq_arg2f_140_outer_knuckle.stl
│   │   │       ├── robotiq_arg2f_base_link.stl
│   │   │       └── robotiq_arg2f_coupling.stl
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── urdf
│   │   │   ├── robotiq_arg2f_140_model_macro.xacro
│   │   │   ├── robotiq_arg2f_140_model.xacro
│   │   │   ├── robotiq_arg2f_transmission.xacro
│   │   │   └── robotiq_arg2f.xacro
│   │   └── visualize.rviz
│   ├── robotiq_2f_85_gripper_visualization
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── test_2f_85_model.launch
│   │   ├── meshes
│   │   │   ├── collision
│   │   │   │   ├── robotiq_arg2f_85_base_link.stl
│   │   │   │   ├── robotiq_arg2f_85_inner_finger.dae
│   │   │   │   ├── robotiq_arg2f_85_inner_knuckle.dae
│   │   │   │   ├── robotiq_arg2f_85_outer_finger.dae
│   │   │   │   ├── robotiq_arg2f_85_outer_knuckle.dae
│   │   │   │   └── robotiq_arg2f_base_link.stl
│   │   │   └── visual
│   │   │       ├── robotiq_arg2f_85_base_link.dae
│   │   │       ├── robotiq_arg2f_85_inner_finger.dae
│   │   │       ├── robotiq_arg2f_85_inner_knuckle.dae
│   │   │       ├── robotiq_arg2f_85_outer_finger.dae
│   │   │       ├── robotiq_arg2f_85_outer_knuckle.dae
│   │   │       ├── robotiq_arg2f_85_pad.dae
│   │   │       └── robotiq_gripper_coupling.stl
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── urdf
│   │   │   ├── robotiq_arg2f_85_model_macro.xacro
│   │   │   ├── robotiq_arg2f_85_model.xacro
│   │   │   ├── robotiq_arg2f_transmission.xacro
│   │   │   └── robotiq_arg2f.xacro
│   │   └── visualize.rviz
│   ├── robotiq_2f_gripper_control
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── robotiq_2f_gripper_control
│   │   │       └── robotiq_gripper_client.h
│   │   ├── launch
│   │   │   ├── robotiq_action_server.launch
│   │   │   ├── robotiq_dual_action_server.launch
│   │   │   ├── test_140mm_gripper.launch
│   │   │   └── test_85mm_gripper.launch
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   ├── robotiq_2f_action_client_example.py
│   │   │   └── robotiq_2f_action_server.py
│   │   ├── setup.py
│   │   └── src
│   │       └── robotiq_2f_gripper_control
│   │           ├── __init__.py
│   │           ├── modbus_crc.py
│   │           ├── robotiq_2f_gripper_driver.py
│   │           └── robotiq_2f_gripper.py
│   ├── robotiq_2f_gripper_msgs
│   │   ├── action
│   │   │   └── CommandRobotiqGripper.action
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── RobotiqGripperCommand.msg
│   │   │   └── RobotiqGripperStatus.msg
│   │   └── package.xml
│   └── robotiq_modbus_rtu
│       ├── CMakeLists.txt
│       ├── mainpage.dox
│       ├── package.xml
│       ├── setup.py
│       └── src
│           └── robotiq_modbus_rtu
│               ├── comModbusRtu.py
│               └── __init__.py
├── robotiq_85_gripper
│   ├── LICENSE
│   ├── README.md
│   ├── robotiq_85_bringup
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── robotiq_85.launch
│   │   │   ├── robotiq_85_test_close.launch
│   │   │   └── robotiq_85_test_open.launch
│   │   ├── package.xml
│   │   └── rviz
│   │       └── robotiq_85.rviz
│   ├── robotiq_85_description
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── display.launch
│   │   │   └── upload_robotiq_85_gripper.launch
│   │   ├── meshes
│   │   │   ├── collision
│   │   │   │   ├── robotiq_85_base_link.stl
│   │   │   │   ├── robotiq_85_finger_link.stl
│   │   │   │   ├── robotiq_85_finger_tip_link.stl
│   │   │   │   ├── robotiq_85_inner_knuckle_link.stl
│   │   │   │   └── robotiq_85_knuckle_link.stl
│   │   │   └── visual
│   │   │       ├── robotiq_85_base_link.dae
│   │   │       ├── robotiq_85_finger_link.dae
│   │   │       ├── robotiq_85_finger_tip_link.dae
│   │   │       ├── robotiq_85_inner_knuckle_link.dae
│   │   │       └── robotiq_85_knuckle_link.dae
│   │   ├── package.xml
│   │   ├── urdf
│   │   │   ├── left_robotiq_85_gripper.xacro
│   │   │   ├── right_robotiq_85_gripper.xacro
│   │   │   ├── robotiq_85_gripper_sim_base.urdf.xacro
│   │   │   ├── robotiq_85_gripper.transmission.xacro
│   │   │   ├── robotiq_85_gripper.urdf.xacro
│   │   │   └── robotiq_85_gripper.xacro
│   │   └── urdf.rviz
│   ├── robotiq_85_driver
│   │   ├── bin
│   │   │   ├── robotiq_85_driver
│   │   │   ├── robotiq_85_test
│   │   │   └── robotiq_85_test_close
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── src
│   │       └── robotiq_85
│   │           ├── gripper_io.py
│   │           ├── __init__.py
│   │           ├── modbus_crc.py
│   │           ├── robotiq_85_driver.py
│   │           ├── robotiq_85_gripper.py
│   │           ├── robotiq_85_gripper_test.py
│   │           ├── robotiq_85_gripper_test.pyc
│   │           └── robotiq_85_test_close.py
│   ├── robotiq_85_gripper
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robotiq_85_moveit_config
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── robotiq_85_gripper.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── joystick_control.launch
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── robotiq_85_gripper_moveit_controller_manager.launch.xml
│   │   │   ├── robotiq_85_gripper_moveit_sensor_manager.launch.xml
│   │   │   ├── robotiq_85_moveit_planning_execution.launch
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   └── package.xml
│   ├── robotiq_85_msgs
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── GripperCmd.msg
│   │   │   └── GripperStat.msg
│   │   └── package.xml
│   ├── robotiq_85_simulation
│   │   ├── roboticsgroup_gazebo_plugins
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include
│   │   │   │   └── roboticsgroup_gazebo_plugins
│   │   │   │       ├── disable_link_plugin.h
│   │   │   │       └── mimic_joint_plugin.h
│   │   │   ├── package.xml
│   │   │   ├── README.md
│   │   │   └── src
│   │   │       ├── disable_link_plugin.cpp
│   │   │       └── mimic_joint_plugin.cpp
│   │   ├── robotiq_85_gazebo
│   │   │   ├── CMakeLists.txt
│   │   │   ├── controller
│   │   │   │   ├── gripper_controller_robotiq_double.yaml
│   │   │   │   ├── gripper_controller_robotiq.yaml
│   │   │   │   └── joint_state_controller.yaml
│   │   │   ├── launch
│   │   │   │   ├── controller_utils.launch
│   │   │   │   ├── robotiq_85.launch
│   │   │   │   ├── robotiq_85_moveit_rviz_test.launch
│   │   │   │   ├── robotiq_85_moveit_sim.launch
│   │   │   │   └── test_kinematics.launch
│   │   │   ├── package.xml
│   │   │   └── scripts
│   │   │       └── robotiq_85_moveit_test.py
│   │   └── robotiq_85_simulation
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   └── si_utils
│       ├── CMakeLists.txt
│       ├── launch
│       │   └── test.launch
│       ├── package.xml
│       └── scripts
│           └── timed_roslaunch
├── ros_control
│   ├── combined_robot_hw
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── etc
│   │   │   └── architecture.svg
│   │   ├── include
│   │   │   └── combined_robot_hw
│   │   │       └── combined_robot_hw.h
│   │   ├── package.xml
│   │   └── src
│   │       └── combined_robot_hw.cpp
│   ├── combined_robot_hw_tests
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── combined_robot_hw_tests
│   │   │       ├── my_robot_hw_1.h
│   │   │       ├── my_robot_hw_2.h
│   │   │       ├── my_robot_hw_3.h
│   │   │       └── my_robot_hw_4.h
│   │   ├── package.xml
│   │   ├── src
│   │   │   ├── dummy_app.cpp
│   │   │   ├── my_robot_hw_1.cpp
│   │   │   ├── my_robot_hw_2.cpp
│   │   │   ├── my_robot_hw_3.cpp
│   │   │   └── my_robot_hw_4.cpp
│   │   ├── test
│   │   │   ├── cm_test.cpp
│   │   │   ├── cm_test.test
│   │   │   ├── combined_robot_hw_test.cpp
│   │   │   └── combined_robot_hw_test.test
│   │   └── test_robot_hw_plugin.xml
│   ├── controller_interface
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── controller_interface
│   │   │       ├── controller_base.h
│   │   │       ├── controller.h
│   │   │       └── multi_interface_controller.h
│   │   ├── package.xml
│   │   └── rosdoc.yaml
│   ├── controller_manager
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── controller_manager
│   │   │       ├── controller_loader.h
│   │   │       ├── controller_loader_interface.h
│   │   │       ├── controller_manager.h
│   │   │       └── controller_spec.h
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── rosdoc.yaml
│   │   ├── scripts
│   │   │   ├── controller_manager
│   │   │   ├── spawner
│   │   │   └── unspawner
│   │   ├── setup.py
│   │   ├── src
│   │   │   ├── controller_manager
│   │   │   │   ├── controller_manager_interface.py
│   │   │   │   ├── controller_manager_interface.pyc
│   │   │   │   └── __init__.py
│   │   │   └── controller_manager.cpp
│   │   └── test
│   │       ├── hwi_switch_test.cpp
│   │       ├── hwi_switch_test.test
│   │       └── hwi_switch_test.yaml
│   ├── controller_manager_msgs
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── ControllersStatistics.msg
│   │   │   ├── ControllerState.msg
│   │   │   ├── ControllerStatistics.msg
│   │   │   └── HardwareInterfaceResources.msg
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   ├── setup.py
│   │   ├── src
│   │   │   └── controller_manager_msgs
│   │   │       ├── __init__.py
│   │   │       ├── utils.py
│   │   │       └── utils.pyc
│   │   └── srv
│   │       ├── ListControllers.srv
│   │       ├── ListControllerTypes.srv
│   │       ├── LoadController.srv
│   │       ├── ReloadControllerLibraries.srv
│   │       ├── SwitchController.srv
│   │       └── UnloadController.srv
│   ├── controller_manager_tests
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── controller_manager_tests
│   │   │       ├── effort_test_controller.h
│   │   │       ├── my_dummy_controller.h
│   │   │       ├── my_robot_hw.h
│   │   │       ├── pos_eff_controller.h
│   │   │       ├── pos_eff_opt_controller.h
│   │   │       └── vel_eff_controller.h
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── src
│   │   │   ├── controller_manager_tests
│   │   │   │   ├── controller_manager_dummy.py
│   │   │   │   └── __init__.py
│   │   │   ├── dummy_app.cpp
│   │   │   ├── effort_test_controller.cpp
│   │   │   ├── my_dummy_controller.cpp
│   │   │   ├── my_robot_hw.cpp
│   │   │   ├── pos_eff_controller.cpp
│   │   │   ├── pos_eff_opt_controller.cpp
│   │   │   └── vel_eff_controller.cpp
│   │   ├── test
│   │   │   ├── cm_msgs_utils_rostest.py
│   │   │   ├── cm_msgs_utils_rostest.test
│   │   │   ├── cm_msgs_utils_test.py
│   │   │   ├── cm_test.cpp
│   │   │   ├── cm_test.test
│   │   │   ├── controller_manager_interface_test.py
│   │   │   ├── controller_manager_scripts.py
│   │   │   ├── controller_manager_scripts.test
│   │   │   ├── controller_params.yaml
│   │   │   └── multi_cm_dummy.py
│   │   └── test_controllers_plugin.xml
│   ├── docs
│   │   └── architecture.svg
│   ├── hardware_interface
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── hardware_interface
│   │   │       ├── actuator_command_interface.h
│   │   │       ├── actuator_state_interface.h
│   │   │       ├── controller_info.h
│   │   │       ├── force_torque_sensor_interface.h
│   │   │       ├── hardware_interface.h
│   │   │       ├── imu_sensor_interface.h
│   │   │       ├── interface_resources.h
│   │   │       ├── internal
│   │   │       │   ├── demangle_symbol.h
│   │   │       │   ├── hardware_resource_manager.h
│   │   │       │   ├── interface_manager.h
│   │   │       │   └── resource_manager.h
│   │   │       ├── joint_command_interface.h
│   │   │       ├── joint_state_interface.h
│   │   │       ├── posvelacc_command_interface.h
│   │   │       ├── posvel_command_interface.h
│   │   │       └── robot_hw.h
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   └── test
│   │       ├── actuator_command_interface_test.cpp
│   │       ├── actuator_state_interface_test.cpp
│   │       ├── force_torque_sensor_interface_test.cpp
│   │       ├── hardware_resource_manager_test.cpp
│   │       ├── imu_sensor_interface_test.cpp
│   │       ├── interface_manager_test.cpp
│   │       ├── joint_command_interface_test.cpp
│   │       ├── joint_state_interface_test.cpp
│   │       ├── posvelacc_command_interface_test.cpp
│   │       ├── posvel_command_interface_test.cpp
│   │       └── robot_hw_test.cpp
│   ├── joint_limits_interface
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── joint_limits_interface
│   │   │       ├── joint_limits.h
│   │   │       ├── joint_limits_interface_exception.h
│   │   │       ├── joint_limits_interface.h
│   │   │       ├── joint_limits_rosparam.h
│   │   │       └── joint_limits_urdf.h
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── rosdoc.yaml
│   │   └── test
│   │       ├── joint_limits_interface_test.cpp
│   │       ├── joint_limits_rosparam.test
│   │       ├── joint_limits_rosparam_test.cpp
│   │       ├── joint_limits_rosparam.yaml
│   │       └── joint_limits_urdf_test.cpp
│   ├── LICENSE
│   ├── README.md
│   ├── ros_control
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── documentation
│   │   │   ├── gazebo_ros_control.odg
│   │   │   ├── gazebo_ros_control.pdf
│   │   │   └── gazebo_ros_control.png
│   │   └── package.xml
│   ├── ros_control.rosinstall
│   ├── rqt_controller_manager
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── plugin.xml
│   │   ├── resource
│   │   │   ├── cm_icon.png
│   │   │   ├── controller_info.ui
│   │   │   ├── controller_manager.ui
│   │   │   ├── led_green.png
│   │   │   ├── led_off.png
│   │   │   └── led_red.png
│   │   ├── scripts
│   │   │   └── rqt_controller_manager
│   │   ├── setup.py
│   │   └── src
│   │       └── rqt_controller_manager
│   │           ├── controller_manager.py
│   │           ├── controller_manager.pyc
│   │           ├── __init__.py
│   │           ├── update_combo.py
│   │           └── update_combo.pyc
│   └── transmission_interface
│       ├── CHANGELOG.rst
│       ├── CMakeLists.txt
│       ├── images
│       │   ├── differential_transmission.png
│       │   ├── differential_transmission.svg
│       │   ├── four_bar_linkage_transmission.png
│       │   ├── four_bar_linkage_transmission.svg
│       │   ├── simple_transmission_gears.png
│       │   ├── simple_transmission.png
│       │   ├── simple_transmission.svg
│       │   └── simple_transmission_timing_belt.png
│       ├── include
│       │   └── transmission_interface
│       │       ├── bidirectional_effort_joint_interface_provider.h
│       │       ├── bidirectional_position_joint_interface_provider.h
│       │       ├── bidirectional_velocity_joint_interface_provider.h
│       │       ├── differential_transmission.h
│       │       ├── differential_transmission_loader.h
│       │       ├── effort_joint_interface_provider.h
│       │       ├── four_bar_linkage_transmission.h
│       │       ├── four_bar_linkage_transmission_loader.h
│       │       ├── joint_state_interface_provider.h
│       │       ├── position_joint_interface_provider.h
│       │       ├── robot_transmissions.h
│       │       ├── simple_transmission.h
│       │       ├── simple_transmission_loader.h
│       │       ├── transmission.h
│       │       ├── transmission_info.h
│       │       ├── transmission_interface_exception.h
│       │       ├── transmission_interface.h
│       │       ├── transmission_interface_loader.h
│       │       ├── transmission_loader.h
│       │       ├── transmission_parser.h
│       │       └── velocity_joint_interface_provider.h
│       ├── mainpage.dox
│       ├── package.xml
│       ├── README.md
│       ├── ros_control_plugins.xml
│       ├── rosdoc.yaml
│       ├── src
│       │   ├── bidirectional_effort_joint_interface_provider.cpp
│       │   ├── bidirectional_position_joint_interface_provider.cpp
│       │   ├── bidirectional_velocity_joint_interface_provider.cpp
│       │   ├── differential_transmission_loader.cpp
│       │   ├── effort_joint_interface_provider.cpp
│       │   ├── four_bar_linkage_transmission_loader.cpp
│       │   ├── joint_state_interface_provider.cpp
│       │   ├── position_joint_interface_provider.cpp
│       │   ├── simple_transmission_loader.cpp
│       │   ├── transmission_interface_loader.cpp
│       │   ├── transmission_loader.cpp
│       │   ├── transmission_parser.cpp
│       │   └── velocity_joint_interface_provider.cpp
│       └── test
│           ├── differential_transmission_loader_test.cpp
│           ├── differential_transmission_test.cpp
│           ├── four_bar_linkage_transmission_loader_test.cpp
│           ├── four_bar_linkage_transmission_test.cpp
│           ├── loader_utils.h
│           ├── random_generator_utils.h
│           ├── read_file.h
│           ├── simple_transmission_loader_test.cpp
│           ├── simple_transmission_test.cpp
│           ├── transmission_interface_loader_test.cpp
│           ├── transmission_interface_test.cpp
│           ├── transmission_parser_test.cpp
│           └── urdf
│               ├── differential_transmission_loader_full.urdf
│               ├── differential_transmission_loader_invalid.urdf
│               ├── differential_transmission_loader_minimal.urdf
│               ├── four_bar_linkage_transmission_loader_full.urdf
│               ├── four_bar_linkage_transmission_loader_invalid.urdf
│               ├── four_bar_linkage_transmission_loader_minimal.urdf
│               ├── parser_test_invalid.urdf
│               ├── parser_test_valid.urdf
│               ├── simple_transmission_loader_full.urdf
│               ├── simple_transmission_loader_invalid.urdf
│               ├── simple_transmission_loader_minimal.urdf
│               ├── transmission_interface_loader_bidirectional_valid.urdf
│               ├── transmission_interface_loader_duplicate.urdf
│               ├── transmission_interface_loader_hw_iface_permutation.urdf
│               ├── transmission_interface_loader_unsupported.urdf
│               └── transmission_interface_loader_valid.urdf
├── roslint
│   ├── CHANGELOG.rst
│   ├── cmake
│   │   └── roslint-extras.cmake.em
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── scripts
│   │   └── test_wrapper
│   ├── setup.py
│   ├── src
│   │   └── roslint
│   │       ├── cpplint.py
│   │       ├── cpplint_wrapper.py
│   │       ├── __init__.py
│   │       ├── pycodestyle.py
│   │       ├── pycodestyle_wrapper.py
│   │       └── README.md
│   └── tests
│       ├── clean1.py
│       ├── CMakeLists.txt
│       ├── dirty1.py
│       └── runlint.py
├── universal_robot
│   ├── CONTRIBUTING.md
│   ├── README.md
│   ├── universal_robot
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── universal_robots
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── ur10_e_moveit_config
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ur10e.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── ur10_e_moveit_controller_manager.launch.xml
│   │   │   ├── ur10_e_moveit_planning_execution.launch
│   │   │   ├── ur10_e_moveit_sensor_manager.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   ├── package.xml
│   │   └── tests
│   │       └── moveit_planning_execution.xml
│   ├── ur10_moveit_config
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ur10.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── ur10_moveit_controller_manager.launch.xml
│   │   │   ├── ur10_moveit_planning_execution.launch
│   │   │   ├── ur10_moveit_sensor_manager.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   ├── package.xml
│   │   └── tests
│   │       └── moveit_planning_execution.xml
│   ├── ur3_e_moveit_config
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ur3e.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── joystick_control.launch
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── ur3_e_moveit_controller_manager.launch.xml
│   │   │   ├── ur3_e_moveit_planning_execution.launch
│   │   │   ├── ur3_e_moveit_sensor_manager.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   ├── package.xml
│   │   └── tests
│   │       └── moveit_planning_execution.xml
│   ├── ur3_moveit_config
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ur3.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── joystick_control.launch
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── ur3_moveit_controller_manager.launch.xml
│   │   │   ├── ur3_moveit_planning_execution.launch
│   │   │   ├── ur3_moveit_sensor_manager.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   ├── package.xml
│   │   └── tests
│   │       └── moveit_planning_execution.xml
│   ├── ur5_e_moveit_config
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ur5e.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── ur5_e_moveit_controller_manager.launch.xml
│   │   │   ├── ur5_e_moveit_planning_execution.launch
│   │   │   ├── ur5_e_moveit_sensor_manager.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   ├── package.xml
│   │   └── tests
│   │       └── moveit_planning_execution.xml
│   ├── ur5_moveit_config
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── controllers.yaml
│   │   │   ├── fake_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── ompl_planning.yaml
│   │   │   └── ur5.srdf
│   │   ├── launch
│   │   │   ├── default_warehouse_db.launch
│   │   │   ├── demo.launch
│   │   │   ├── fake_moveit_controller_manager.launch.xml
│   │   │   ├── move_group.launch
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_rviz.launch
│   │   │   ├── ompl_planning_pipeline.launch.xml
│   │   │   ├── planning_context.launch
│   │   │   ├── planning_pipeline.launch.xml
│   │   │   ├── run_benchmark_ompl.launch
│   │   │   ├── sensor_manager.launch.xml
│   │   │   ├── setup_assistant.launch
│   │   │   ├── trajectory_execution.launch.xml
│   │   │   ├── ur5_moveit_controller_manager.launch.xml
│   │   │   ├── ur5_moveit_planning_execution.launch
│   │   │   ├── ur5_moveit_sensor_manager.launch.xml
│   │   │   ├── warehouse.launch
│   │   │   └── warehouse_settings.launch.xml
│   │   ├── package.xml
│   │   └── tests
│   │       └── moveit_planning_execution.xml
│   ├── ur_bringup
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── ur10_bringup_joint_limited.launch
│   │   │   ├── ur10_bringup.launch
│   │   │   ├── ur3_bringup_joint_limited.launch
│   │   │   ├── ur3_bringup.launch
│   │   │   ├── ur5_bringup_joint_limited.launch
│   │   │   ├── ur5_bringup.launch
│   │   │   └── ur_common.launch
│   │   ├── package.xml
│   │   └── tests
│   │       └── roslaunch_test.xml
│   ├── ur_description
│   │   ├── cfg
│   │   │   └── view_robot.rviz
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── ur10_upload.launch
│   │   │   ├── ur3_upload.launch
│   │   │   ├── ur5_upload.launch
│   │   │   ├── view_ur10.launch
│   │   │   ├── view_ur3.launch
│   │   │   └── view_ur5.launch
│   │   ├── meshes
│   │   │   ├── ur10
│   │   │   │   ├── collision
│   │   │   │   │   ├── base.stl
│   │   │   │   │   ├── forearm.stl
│   │   │   │   │   ├── shoulder.stl
│   │   │   │   │   ├── upperarm.stl
│   │   │   │   │   ├── wrist1.stl
│   │   │   │   │   ├── wrist2.stl
│   │   │   │   │   └── wrist3.stl
│   │   │   │   └── visual
│   │   │   │       ├── base.dae
│   │   │   │       ├── forearm.dae
│   │   │   │       ├── shoulder.dae
│   │   │   │       ├── upperarm.dae
│   │   │   │       ├── wrist1.dae
│   │   │   │       ├── wrist2.dae
│   │   │   │       └── wrist3.dae
│   │   │   ├── ur3
│   │   │   │   ├── collision
│   │   │   │   │   ├── base.stl
│   │   │   │   │   ├── forearm.stl
│   │   │   │   │   ├── shoulder.stl
│   │   │   │   │   ├── upperarm.stl
│   │   │   │   │   ├── wrist1.stl
│   │   │   │   │   ├── wrist2.stl
│   │   │   │   │   └── wrist3.stl
│   │   │   │   └── visual
│   │   │   │       ├── base.dae
│   │   │   │       ├── forearm.dae
│   │   │   │       ├── shoulder.dae
│   │   │   │       ├── upperarm.dae
│   │   │   │       ├── wrist1.dae
│   │   │   │       ├── wrist2.dae
│   │   │   │       └── wrist3.dae
│   │   │   └── ur5
│   │   │       ├── collision
│   │   │       │   ├── base.stl
│   │   │       │   ├── forearm.stl
│   │   │       │   ├── shoulder.stl
│   │   │       │   ├── upperarm.stl
│   │   │       │   ├── wrist1.stl
│   │   │       │   ├── wrist2.stl
│   │   │       │   └── wrist3.stl
│   │   │       └── visual
│   │   │           ├── base.dae
│   │   │           ├── forearm.dae
│   │   │           ├── shoulder.dae
│   │   │           ├── upperarm.dae
│   │   │           ├── wrist1.dae
│   │   │           ├── wrist2.dae
│   │   │           └── wrist3.dae
│   │   ├── model.pdf
│   │   ├── package.xml
│   │   └── urdf
│   │       ├── common.gazebo.xacro
│   │       ├── ur10_joint_limited_robot.urdf.xacro
│   │       ├── ur10_robot.urdf.xacro
│   │       ├── ur10.urdf.xacro
│   │       ├── ur3_joint_limited_robot.urdf.xacro
│   │       ├── ur3_robot.urdf.xacro
│   │       ├── ur3.urdf.xacro
│   │       ├── ur5_joint_limited_robot.urdf.xacro
│   │       ├── ur5_robot.urdf.xacro
│   │       ├── ur5.urdf.xacro
│   │       ├── ur.gazebo.xacro
│   │       └── ur.transmission.xacro
│   ├── ur_driver
│   │   ├── cfg
│   │   │   └── URDriver.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── prog
│   │   ├── prog_reset
│   │   ├── setup.py
│   │   ├── src
│   │   │   └── ur_driver
│   │   │       ├── deserialize.py
│   │   │       ├── deserializeRT.py
│   │   │       ├── driver.py
│   │   │       ├── __init__.py
│   │   │       ├── io_interface.py
│   │   │       ├── test_comm.py
│   │   │       └── testRT_comm.py
│   │   ├── test_io.py
│   │   └── test_move.py
│   ├── ur_e_description
│   │   ├── cfg
│   │   │   └── view_robot.rviz
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── ur10e_upload.launch
│   │   │   ├── ur3e_upload.launch
│   │   │   ├── ur5e_upload.launch
│   │   │   ├── view_ur10e.launch
│   │   │   ├── view_ur3e.launch
│   │   │   └── view_ur5e.launch
│   │   ├── meshes
│   │   │   ├── ur10e
│   │   │   │   ├── collision
│   │   │   │   │   ├── base.stl
│   │   │   │   │   ├── forearm.stl
│   │   │   │   │   ├── shoulder.stl
│   │   │   │   │   ├── upperarm.stl
│   │   │   │   │   ├── wrist1.stl
│   │   │   │   │   ├── wrist2.stl
│   │   │   │   │   └── wrist3.stl
│   │   │   │   └── visual
│   │   │   │       ├── base.dae
│   │   │   │       ├── forearm.dae
│   │   │   │       ├── shoulder.dae
│   │   │   │       ├── upperarm.dae
│   │   │   │       ├── wrist1.dae
│   │   │   │       ├── wrist2.dae
│   │   │   │       └── wrist3.dae
│   │   │   ├── ur3e
│   │   │   │   ├── collision
│   │   │   │   │   ├── base.stl
│   │   │   │   │   ├── forearm.stl
│   │   │   │   │   ├── shoulder.stl
│   │   │   │   │   ├── upperarm.stl
│   │   │   │   │   ├── wrist1.stl
│   │   │   │   │   ├── wrist2.stl
│   │   │   │   │   └── wrist3.stl
│   │   │   │   └── visual
│   │   │   │       ├── base.dae
│   │   │   │       ├── forearm.dae
│   │   │   │       ├── shoulder.dae
│   │   │   │       ├── upperarm.dae
│   │   │   │       ├── wrist1.dae
│   │   │   │       ├── wrist2.dae
│   │   │   │       └── wrist3.dae
│   │   │   └── ur5e
│   │   │       ├── collision
│   │   │       │   ├── base.stl
│   │   │       │   ├── forearm.stl
│   │   │       │   ├── shoulder.stl
│   │   │       │   ├── upperarm.stl
│   │   │       │   ├── wrist1.stl
│   │   │       │   ├── wrist2.stl
│   │   │       │   └── wrist3.stl
│   │   │       └── visual
│   │   │           ├── base.dae
│   │   │           ├── forearm.dae
│   │   │           ├── shoulder.dae
│   │   │           ├── upperarm.dae
│   │   │           ├── wrist1.dae
│   │   │           ├── wrist2.dae
│   │   │           └── wrist3.dae
│   │   ├── package.xml
│   │   └── urdf
│   │       ├── common.gazebo.xacro
│   │       ├── ur10e_joint_limited_robot.urdf.xacro
│   │       ├── ur10e_robot.urdf.xacro
│   │       ├── ur10e.urdf.xacro
│   │       ├── ur3e_joint_limited_robot.urdf.xacro
│   │       ├── ur3e_robot.urdf.xacro
│   │       ├── ur3e.urdf.xacro
│   │       ├── ur5e_joint_limited_robot.urdf.xacro
│   │       ├── ur5e_robot.urdf.xacro
│   │       ├── ur5e.urdf.xacro
│   │       ├── ur.gazebo.xacro
│   │       └── ur.transmission.xacro
│   ├── ur_e_gazebo
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── controller
│   │   │   ├── arm_controller_ur10e.yaml
│   │   │   ├── arm_controller_ur3e.yaml
│   │   │   ├── arm_controller_ur5e.yaml
│   │   │   └── joint_state_controller.yaml
│   │   ├── launch
│   │   │   ├── controller_utils.launch
│   │   │   ├── ur10e_joint_limited.launch
│   │   │   ├── ur10e.launch
│   │   │   ├── ur3e_joint_limited.launch
│   │   │   ├── ur3e.launch
│   │   │   ├── ur5e_joint_limited.launch
│   │   │   └── ur5e.launch
│   │   ├── package.xml
│   │   └── tests
│   │       └── roslaunch_test.xml
│   ├── ur_gazebo
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── controller
│   │   │   ├── arm_controller_ur10.yaml
│   │   │   ├── arm_controller_ur3.yaml
│   │   │   ├── arm_controller_ur5.yaml
│   │   │   └── joint_state_controller.yaml
│   │   ├── launch
│   │   │   ├── controller_utils.launch
│   │   │   ├── ur10_joint_limited.launch
│   │   │   ├── ur10.launch
│   │   │   ├── ur3_joint_limited.launch
│   │   │   ├── ur3.launch
│   │   │   ├── ur5_joint_limited.launch
│   │   │   └── ur5.launch
│   │   ├── package.xml
│   │   └── tests
│   │       └── roslaunch_test.xml
│   ├── ur_kinematics
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── ur_kinematics
│   │   │       ├── ikfast.h
│   │   │       ├── ur_kin.h
│   │   │       └── ur_moveit_plugin.h
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── src
│   │   │   ├── ur_kin.cpp
│   │   │   ├── ur_kinematics
│   │   │   │   ├── __init__.py
│   │   │   │   └── test_analytical_ik.py
│   │   │   ├── ur_kin_py.cpp
│   │   │   └── ur_moveit_plugin.cpp
│   │   └── ur_moveit_plugins.xml
│   └── ur_msgs
│       ├── CHANGELOG.rst
│       ├── CMakeLists.txt
│       ├── msg
│       │   ├── Analog.msg
│       │   ├── Digital.msg
│       │   ├── IOStates.msg
│       │   ├── MasterboardDataMsg.msg
│       │   ├── RobotModeDataMsg.msg
│       │   ├── RobotStateRTMsg.msg
│       │   └── ToolDataMsg.msg
│       ├── package.xml
│       └── srv
│           ├── SetIO.srv
│           ├── SetPayload.srv
│           └── SetSpeedSliderFraction.srv
└── ur_modern_driver
    ├── CMakeLists.txt
    ├── config
    │   ├── ur10_controllers.yaml
    │   ├── ur3_controllers.yaml
    │   └── ur5_controllers.yaml
    ├── include
    │   └── ur_modern_driver
    │       ├── bin_parser.h
    │       ├── event_counter.h
    │       ├── log.h
    │       ├── pipeline.h
    │       ├── queue
    │       │   ├── atomicops.h
    │       │   ├── LICENSE.md
    │       │   └── readerwriterqueue.h
    │       ├── ros
    │       │   ├── action_server.h
    │       │   ├── action_trajectory_follower_interface.h
    │       │   ├── controller.h
    │       │   ├── hardware_interface.h
    │       │   ├── io_service.h
    │       │   ├── lowbandwidth_trajectory_follower.h
    │       │   ├── mb_publisher.h
    │       │   ├── rt_publisher.h
    │       │   ├── service_stopper.h
    │       │   ├── trajectory_follower.h
    │       │   └── urscript_handler.h
    │       ├── tcp_socket.h
    │       ├── test
    │       │   ├── random_data.h
    │       │   └── utils.h
    │       ├── types.h
    │       └── ur
    │           ├── commander.h
    │           ├── consumer.h
    │           ├── factory.h
    │           ├── master_board.h
    │           ├── messages.h
    │           ├── messages_parser.h
    │           ├── parser.h
    │           ├── producer.h
    │           ├── robot_mode.h
    │           ├── rt_parser.h
    │           ├── rt_state.h
    │           ├── server.h
    │           ├── state.h
    │           ├── state_parser.h
    │           └── stream.h
    ├── launch
    │   ├── ur10_bringup_compatible.launch
    │   ├── ur10_bringup_joint_limited.launch
    │   ├── ur10_bringup.launch
    │   ├── ur10_ros_control.launch
    │   ├── ur3_bringup_joint_limited.launch
    │   ├── ur3_bringup.launch
    │   ├── ur3_ros_control.launch
    │   ├── ur5_bringup_compatible.launch
    │   ├── ur5_bringup_joint_limited.launch
    │   ├── ur5_bringup.launch
    │   ├── ur5_ros_control.launch
    │   └── ur_common.launch
    ├── LICENSE
    ├── package.xml
    ├── README.md
    ├── src
    │   ├── ros
    │   │   ├── action_server.cpp
    │   │   ├── controller.cpp
    │   │   ├── hardware_interface.cpp
    │   │   ├── lowbandwidth_trajectory_follower.cpp
    │   │   ├── mb_publisher.cpp
    │   │   ├── rt_publisher.cpp
    │   │   ├── service_stopper.cpp
    │   │   ├── trajectory_follower.cpp
    │   │   └── urscript_handler.cpp
    │   ├── ros_main.cpp
    │   ├── tcp_socket.cpp
    │   └── ur
    │       ├── commander.cpp
    │       ├── master_board.cpp
    │       ├── messages.cpp
    │       ├── robot_mode.cpp
    │       ├── rt_state.cpp
    │       ├── server.cpp
    │       └── stream.cpp
    ├── test_move.py
    └── tests
        ├── main.cpp
        └── ur
            ├── master_board.cpp
            ├── robot_mode.cpp
            └── rt_state.cpp
  ```
</details>

These *ROS* repositories or packages form the basis for the implementation to be developed, i.e. they serve as system resources.


## Preparation for the implementation of the multi-robot system
Once all the necessary resources for the system are properly installed, we will proceed with the creation of the package that contains the various proposed solutions with their advantages and disadvantages. 

It has been decided to organize the package in such a way that the common resources are shared by the different implementations and the specific changes that need to be made are stored in their respective directories. In this way, the modified files can be run instead of the original files from which they originated, simply by modifying the file contained in the files with the *launch* extension. 

For this reason, we first create the directory that will contain all the proposed solutions:

```bash
cd ~/MultiCobot-UR10-Gripper/src
mkdir multirobot
```

This directory will be the root directory of the deployments and this will complete the preparation for the reproduction of the various solutions. 

Add that the changes that are frequently made to all the solutions will be changed on the shared resources.

--- 

<div>
<p align="left">
<button name="button">
<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup_eng.md"> Menu </a>
</button>
</p>



<p>
<span style="float:left;">
<button name="button">
<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/leap-motion.md"> Previous </a>
</button>
</span>
</p>
</div>