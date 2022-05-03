#This software is distributed under the terms of the MIT License.
#Copyright (c) 2022 107-Systems
#Author: Jonas Wühr

cmake_minimum_required(VERSION 3.0.2)

project(l3xz_openmv_camera)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init -f
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Rgb.srv
  Ir.srv
  Gpio.srv
  GpioSet.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

# Install executable
catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install dependencies
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/camera_interface.py DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/openmv DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib/${PROJECT_NAME})
