cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)

#FIXME: Physics still not implemented
OPTION(BUILD_BULLET_PHYSICS "Add physics support with Bullet? (fix routes in CMakeLists)" OFF)
IF (BUILD_BULLET_PHYSICS)
        #FIXME: Remove hard-coded routes
        #TODO: Create FindXXXX.cmake files instead
        SET(BULLET_INCLUDE_DIR /home/mprats/local/bullet-2.78/include)
        SET(BULLET_LIBRARY_DIR /home/mprats/local/bullet-2.78/build/lib)
        SET(OSGWORKS_INCLUDE_DIR /home/mprats/local/osgWorks/build/include)
        SET(OSGWORKS_LIBRARY_DIR /home/mprats/local/osgWorks/build/lib)
        SET(OSGBULLET_INCLUDE_DIR /home/mprats/local/osgBullet/build/include)
        SET(OSGBULLET_LIBRARY_DIR /home/mprats/local/osgBullet/build/lib)
        SET(BULLET_LIBRARIES osgbBullet osgwControls osgwTools BulletCollision LinearMath BulletDynamics)
ENDIF()

SET(SIMULATOR_ROOT_PATH "${PROJECT_SOURCE_DIR}/")
SET(SIMULATOR_DATA_PATH "${PROJECT_SOURCE_DIR}/data/")

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/src/SimulatorConfig.h.cmake
  ${PROJECT_SOURCE_DIR}/src/SimulatorConfig.h
)

pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIR} ${OSGWORKS_INCLUDE_DIR} ${OSGBULLET_INCLUDE_DIR} ${LIBXML++_INCLUDE_DIRS})
LINK_DIRECTORIES(${BULLET_LIBRARY_DIR} ${OSGWORKS_LIBRARY_DIR} ${OSGBULLET_LIBRARY_DIR} ${LIBXML++_LIBRARY_DIRS})

# this disables the warnings that come mostly from osg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")

rosbuild_add_library(uwsim 
src/osgOceanScene.cpp
src/SkyDome.cpp
src/SphereSegment.cpp
src/KinematicChain.cpp
src/URDFRobot.cpp
src/SimulatedIAUV.cpp
src/VirtualCamera.cpp
src/VirtualRangeSensor.cpp
src/HUDCamera.cpp
src/ConfigXMLParser.cpp
src/UWSimUtils.cpp
src/ROSInterface.cpp
src/ObjectPicker.cpp
src/SceneBuilder.cpp
src/ViewBuilder.cpp
src/InertialMeasurementUnit.cpp
src/PressureSensor.cpp
src/DVLSensor.cpp
src/GPSSensor.cpp
)

rosbuild_add_executable(UWSim  src/main.cpp)
rosbuild_add_executable(makeVehicleSurvey interface_examples/makeVehicleSurvey.cpp)
rosbuild_add_executable(setJointPosition interface_examples/setJointPosition.cpp)
rosbuild_add_executable(setJointVelocity interface_examples/setJointVelocity.cpp)
rosbuild_add_executable(setVehiclePosition interface_examples/setVehiclePosition.cpp)
rosbuild_add_executable(setVehiclePose interface_examples/setVehiclePose.cpp)
rosbuild_add_executable(setVehicleVelocity interface_examples/setVehicleVelocity.cpp)
rosbuild_add_executable(setVehicleTwist interface_examples/setVehicleTwist.cpp)
rosbuild_add_executable(setVehicleDisturbances interface_examples/setVehicleDisturbances.cpp)
rosbuild_add_executable(gotoAbsolutePose interface_examples/gotoAbsolutePose.cpp)
rosbuild_add_executable(gotoRelativePose interface_examples/gotoRelativePose.cpp)
rosbuild_add_executable(setVehicleActuators interface_examples/setVehicleActuators.cpp)

TARGET_LINK_LIBRARIES(UWSim uwsim ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} GLU)
TARGET_LINK_LIBRARIES(makeVehicleSurvey ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setJointPosition ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setJointVelocity ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setVehiclePose ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setVehiclePosition ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setVehicleVelocity ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setVehicleTwist ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setVehicleDisturbances ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(gotoAbsolutePose ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(gotoRelativePose ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})
TARGET_LINK_LIBRARIES(setVehicleActuators ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES})

