cmake_minimum_required(VERSION 3.12)
project(3D_finger_vein_v3)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
message("=========OpenCV library status:=========")
message("   version:${OpenCV_VERSION}")
message("   libraries:${OpenCV_LIBS}")
message("   include path:${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PythonLibs 2.7)
message("=========PythonLibs library status:=========")
message("   directories:${PYTHON_INCLUDE_DIRS}")
message("   libraries:${PYTHON_LIBRARIES}")
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARIES})

add_executable(3D_finger_vein_v3 main.cpp preprocess/preprocess.cpp preprocess/preprocess.h utils/utils.cpp utils/utils.h
        reconstruction/finger_reconstruct.cpp reconstruction/finger_reconstruct.h edge_detection/edge_detection.cpp
        edge_detection/edge_detection.h reconstruction/ellipse_optimize.cpp reconstruction/ellipse_optimize.h
        reconstruction/sub_ellipse_optimize.cpp reconstruction/sub_ellipse_optimize.h reconstruction/matplotlibcpp.h
        reconstruction/coor_plot.cpp reconstruction/coor_plot.h map_texture/map_texture.cpp map_texture/map_texture.h)

target_link_libraries(3D_finger_vein_v3 ${OpenCV_LIBS})

# 添加glog
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
link_libraries(glog)
target_link_libraries(3D_finger_vein_v3 glog)

# 添加nlopt
target_link_libraries(3D_finger_vein_v3 libnlopt.so.0)

# 添加matplotcpp
target_link_libraries(3D_finger_vein_v3 ${PYTHON_LIBRARIES})

# 添加OpenGL、GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})