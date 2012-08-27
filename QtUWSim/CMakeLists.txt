cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)



MACRO (QT4_WRAP_UI_UWSIM outfiles )
  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

  FOREACH (it ${ui_files})
    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
    SET(outfile ${PROJECT_SOURCE_DIR}/src/ui_${outfile}.h) # Here we set output
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
      COMMAND ${QT_UIC_EXECUTABLE}
      ARGS ${ui_options} -o ${outfile} ${infile}
      MAIN_DEPENDENCY ${infile})
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH (it)

ENDMACRO (QT4_WRAP_UI_UWSIM)



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
set(BUILD_ROS_INTERFACES 1)

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osgManipulator osg)
SET(OSGQT_LIB /usr/lib/libosgQt.so)


pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

FIND_PACKAGE(Qt4 REQUIRED)
set (QT_USE_QTGUI TRUE)
set (QT_USE_QTOPENGL TRUE)
set (QT_USE_QTXML TRUE)
include(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

set(qt_hdrs src/MainWindow.h)
set(forms src/MainWindow.ui)

QT4_WRAP_UI_UWSIM(forms_headers ${forms})


QT4_WRAP_CPP(qt_moc_srcs  ${qt_hdrs} )





INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS} ${QT_LIBRARY_DIR})
LINK_DIRECTORIES(${LIBXML++_LIBRARY_DIRS} ${QT_LIBRARY_DIR})

# this disables the warnings that come mostly from osg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")

rosbuild_add_executable(QtUWSim  src/main.cpp
${qt_moc_srcs}
${forms_headers}
src/MainWindow.cpp
src/MosaicManipulator.cpp
src/PlanarGraspSpec.cpp
src/HandInteractiveMarker.cpp
src/GraspSpecification.cpp
src/CustomTrackballDragger.cpp
src/CustomTabPlaneTrackballDragger.cpp
src/joint_state_listener.cpp
)



TARGET_LINK_LIBRARIES(QtUWSim ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} ${OSGQT_LIB} ${QT_LIBRARIES} GLU sqlite3)
