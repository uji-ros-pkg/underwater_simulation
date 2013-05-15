cmake_minimum_required(VERSION 2.8.3)
project(UWSim)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  image_transport
  uwsim_osgocean
  urdf
  urdf_parser
  underwater_sensor_msgs
  uwsim_osgbullet
  uwsim_osgworks
  resource_retriever
  roscpp
  libxml++-2.6
  boost
  libopenscenegraph
  opengl
)

catkin_package(
  INCLUDE_DIRS
    include  
  LIBRARIES
    uwsim
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    nav_msgs
    image_transport
    uwsim_osgocean
    urdf
    urdf_parser
    underwater_sensor_msgs
    uwsim_osgbullet
    uwsim_osgworks
    resource_retriever
    roscpp
    libxml++-2.6
    boost
    libopenscenegraph
    opengl
)

find_package(Boost REQUIRED thread program_options system filesystem)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)

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

link_directories(${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(uwsim 
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
src/MultibeamSensor.cpp
src/BulletPhysics.cpp
src/PhysicsBuilder.cpp
src/BulletHfFluid/btHfFluid.cpp
src/BulletHfFluid/btHfFluidBuoyantConvexShape.cpp
src/BulletHfFluid/btHfFluidBuoyantShapeCollisionAlgorithm.cpp
src/BulletHfFluid/btHfFluidCollisionShape.cpp
src/BulletHfFluid/btHfFluidRigidCollisionAlgorithm.cpp
src/BulletHfFluid/btHfFluidRigidCollisionConfiguration.cpp
src/BulletHfFluid/btHfFluidRigidDynamicsWorld.cpp
src/BulletHfFluid/BuoyantShapeConvexCollisionAlgorithm.cpp
)

target_link_libraries(uwsim ${catkin_LIBRARIES})
add_dependencies(uwsim underwater_sensor_msgs_gencpp)

add_executable(UWSim_binary  src/main.cpp)
target_link_libraries(UWSim_binary uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(makeVehicleSurvey interface_examples/makeVehicleSurvey.cpp)
target_link_libraries(makeVehicleSurvey uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setJointPosition interface_examples/setJointPosition.cpp)
target_link_libraries(setJointPosition uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setJointVelocity interface_examples/setJointVelocity.cpp)
target_link_libraries(setJointVelocity uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehiclePosition interface_examples/setVehiclePosition.cpp)
target_link_libraries(setVehiclePosition uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehiclePose interface_examples/setVehiclePose.cpp)
target_link_libraries(setVehiclePose uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleVelocity interface_examples/setVehicleVelocity.cpp)
target_link_libraries(setVehicleVelocity uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleTwist interface_examples/setVehicleTwist.cpp)
target_link_libraries(setVehicleTwist uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleDisturbances interface_examples/setVehicleDisturbances.cpp)
target_link_libraries(setVehicleDisturbances uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(gotoAbsolutePose interface_examples/gotoAbsolutePose.cpp)
target_link_libraries(gotoAbsolutePose uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(gotoRelativePose interface_examples/gotoRelativePose.cpp)
target_link_libraries(gotoRelativePose uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleActuators interface_examples/setVehicleActuators.cpp)
target_link_libraries(setVehicleActuators uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    uwsim
    UWSim_binary
    makeVehicleSurvey
    setJointPosition
    setJointVelocity
    setVehiclePosition
    setVehiclePose
    setVehicleVelocity
    setVehicleTwist
    setVehicleDisturbances
    gotoAbsolutePose
    gotoRelativePose
    setVehicleActuators
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   FILES_MATCHING PATTERN "UWSim" 
)
