cmake_minimum_required(VERSION 2.8.3)
project(robotiq_85_driver)

find_package(catkin REQUIRED COMPONENTS roslaunch rospy)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS bin/robotiq_85_driver bin/robotiq_85_test bin/robotiq_85_test_close
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
