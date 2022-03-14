var store = [{
        "title": "Introduction",
        "excerpt":"MultiCobot-UR10-Gripper Multi-robot system of collaborative robots (cobots) UR10s with grippers from Robotiq (robotiq_85_gripper) that allows simultaneous execution of tasks with different types of controllers and brands of cobots, as well as direct control of the cobot by a person via the Leap Motion device. About this project This thesis project...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/introduction/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Requirements",
        "excerpt":"Here is a step by step explanation of how to install ROS Kinetic Kame and all the necessary tools and packages. It is not guaranteed to work after reproducing the project. This is because the installed packages may have been updated and therefore need to be adjusted to the updated...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/requirements/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "ROS Kinetic &amp; dependencies",
        "excerpt":"Installation of ROS Kinetic Kame sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" &gt; /etc/apt/sources.list.d/ros-latest.list' sudo apt install curl # if you haven't already installed curl curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - sudo apt-get update sudo apt-get install ros-kinetic-desktop-full sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/ros-dependencies/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "ROS package",
        "excerpt":"Create working directory mkdir -p ~/ MultiCobot-UR10-Gripper/src cd ~/MultiCobot-UR10-Gripper/src Warning: ROS Third-party packages. No direct changes were made to the contents of ROS packages during the installation process. Any changes made to these packages were to accommodate ROS Kinetic during compilation. The licenses of each installed package have been respected....","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/ros-packages/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Leap Motion's setup",
        "excerpt":"Leap Motion Settings ROS Driver for Leap Motion Controller Proper installation and configuration of the Leap Motion driver in ROS Kinetic Kame requires a bit more work than in the previous repositories. First, you need to replace or create the file that serves the driver: /lib/systemd/system/leapd.service # Founded by Kevin...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-setup/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Testing the setup",
        "excerpt":"If there are warnings during the build Ignored warnings: WARNING: Package 'ur_modern_driver' is deprecated (This package has been deprecated. Users of CB3 and e-Series controllers should migrate to ur_robot_driver.) CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:418 (message): catkin_package() include dir '/home/miguel/MultiCobot-UR10-Gripper/build/gazebo-pkgs/gazebo_grasp_plugin/..' should be placed in the devel space instead of the build space...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-setup-testing/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Design of a multi-robot system",
        "excerpt":"After running numerous tests in the ROS environment to create a multi-robot system, it turns out that the solutions must come from the combinations between the URDF file that defines the robot model and the ROS packages included in the schema which are represented by the MoveIt! package, whose main...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/design-multi-robot-system/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Solution with our motion planning",
        "excerpt":"In the schematic design, the main component that performs the motion planner functions must be implemented from scratch. The motion planner communicates directly with the controllers needed to execute the cobot’s motions, it does not need to be contained in a namespace. You must describe all the cobots you want...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/own-planner-solution/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "One UR10 with gripper",
        "excerpt":"The phases you see in the diagram are for orientation. You can go through them in the order you wish. The diagram has been divided into phases so that you can follow a sequence and know which element of the diagram is being worked on. In this case, it starts...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/own-one/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Two UR10s with grippers",
        "excerpt":"This time the solution for two robots is done in the same way as for one, changing the content of the files and adapting them to the simulation with two robots. The phases you see in the diagram are for orientation. They can be executed in the order you want....","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/own-two/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Four UR10s with grippers",
        "excerpt":"This time the solution for four robots is done in the same way as for two robots, changing the content of the files and adapting them to the simulation with four robots. The phases you see in the diagram are for orientation. They can be executed in the order you...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/own-four/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Solution with MoveIt's motion planning",
        "excerpt":"Index Modeling of multiple robots in the URDF file Replicating the MoveIt! node Modeling of multiple robots in the URDF file In the schematic design, the main component that performs the planner functions is the MoveIt! package. It is based on adding robots in the URDF file and the MoveIt!...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/moveit-motion-planning/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "One UR10 with gripper",
        "excerpt":"Prerequisite Successfully install the Basic System Configuration. Implement the Solution for one robot without the MoveIt! motion planner. Index Phase 1: URDF configuration Phase 2: Configuration of MoveIt! Phase 3: Simulation of a Pick &amp; Place in Gazebo Execution of tests Phase 1: URDF configuration :book: Description of the file...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/moveit-one/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Two UR10s with grippers",
        "excerpt":"This time, the solution for two robots is created in the same way as for one robot. However, the content of the files is changed to adapt them to the simulation with two robots. Prerequisite Successfully install the Basic System Configuration. Implement the Solution for one robot without the MoveIt!...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/moveit-two/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Four UR10s with grippers",
        "excerpt":"The solution is going to be carried out for four robots this time, in the same way that it has been carried out for one, but modifying the content of the files, adapting it for simulation with four robots. Prerequisite Successfully install the Basic System Configuration. Implement the Solution for...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/moveit-four/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Design &amp; Integration",
        "excerpt":"Here we explain how Leap Motion is integrated into the system developed so far to control up to two robots simultaneously for the different solutions proposed in this repository. In the schematic representation of the design, this section represents stages 4 and 5. Prerequisite Successful installation of Basic System Configuration....","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-design-integration/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "One UR10 with gripper &amp; Leap Motion",
        "excerpt":"This section is the continuation of phases 1, 2 and 3, where the integration of Leap Motion into the system takes place, i.e. phases 4 and 5. In the previous phases, the robot was controlled by sending the previously defined trajectories to perform the Pick &amp; Place. Now this is...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-own-one/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Two UR10s with grippers &amp; Leap Motion",
        "excerpt":"This section is the continuation of phases 1, 2 and 3, where the integration of Leap Motion into the system takes place, i.e. phases 4 and 5. In the previous phases, the robots were controlled by sending the previously defined trajectories to perform the pick &amp; place. Now this is...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-own-two/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "One UR10 with gripper &amp; Leap Motion",
        "excerpt":"This section is the continuation of phases 1, 2 and 3, where the integration of Leap Motion into the system takes place, i.e. phases 4 and 5. In the previous phases, the robots were controlled by sending the previously defined trajectories to perform the Pick &amp; Place, but now this...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-moveit-one/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Two UR10s with grippers &amp; Leap Motion",
        "excerpt":"This section is the continuation of phases 1, 2 and 3, where the integration of Leap Motion into the system takes place, i.e. phases 4 and 5. In the previous phases, the robots were controlled by sending the previously defined trajectories to perform the pick &amp; place. Now this is...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/lm-moveit-two/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Robot Campero",
        "excerpt":"Since the internal configuration of the Campero robot system may vary depending on the company that configured it. We will not go into detail on how the system configuration was done. The code serves as a reference that you can modify or take from the implemented code. View on Github...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/robot-campero/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "License",
        "excerpt":"Attribution 4.0 International ======================================================================= Creative Commons Corporation (“Creative Commons”) is not a law firm and does not provide legal services or legal advice. Distribution of Creative Commons public licenses does not create a lawyer-client or other relationship. Creative Commons makes its licenses and related information available on an “as-is” basis....","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/license/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Help &amp; Support",
        "excerpt":"Unfortunately, this repository is not actively maintained. The main goal is to publish what has been learned for the community who might need the knowledge and content of this repository to develop their project or research. A response is not guaranteed, but you can contact the authors using the information...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/help-support/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Authors",
        "excerpt":"Burgh Oliván, Miguel - Author of the Final Degree Project entitled Multirobot system for the collaborative transport of objects. López Nicolás, Gonzalo - Director of the Final Degree Project entitled Multirobot system for the collaborative transport of objects. The memory of the Final Degree Project can be found in the...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/authors/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Acknowledgement",
        "excerpt":"This work is part of the RoPeRT research group of the i3A, the University of Zaragoza. The developed work has been experimentally evaluated and validated, showing a correct operation of the physical robot Campero. For this reason, this work is part of the activities of the project COMMANDIA (2019), co-funded...","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/acknowledgement/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },{
        "title": "Recognition",
        "excerpt":"Please cite this work if the content of this repository has been useful to you:   BibTeX:  @article{     BurghOliván:66296,     author = \"Burgh Oliván, Miguel Yankan and López Nicolás, Gonzalo\",     title = \"{Sistema multirobot para el transporte colaborativo de objetos}\",     year  = \"2022\", }  ","categories": [],
        "tags": [],
        "url": "/MultiCobot-UR10-Gripper/docs/recognition/",
        "teaser": "/MultiCobot-UR10-Gripper/assets/images/proyect-logo-2.png"
      },]
