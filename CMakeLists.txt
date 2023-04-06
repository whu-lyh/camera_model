# CameraModel CMakeLists.txt
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(CameraModel)

OPTION(BUILD_WITH_CERES "Build with CERES for Non-linear Optimization" ON)
OPTION(DEBUG_ON "Turn on DEBUG Mode" OFF)
#OPTION(BUILD_SHARED_LIBS "Build shared libraries" ON)

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC -fopenmp")

IF(DEBUG_ON)
	SET(EXE_NAME CameraModel_d)
ELSE()
	SET(EXE_NAME CameraModel)
ENDIF()

SET(THIRDPARTY_INCLUDE "D:/ThirdPartyLibraries-20210530/include")
IF(DEBUG_ON)
    SET(THIRDPARTY_LIBRARIES "D:/ThirdPartyLibraries-20210530/x64/vc15/lib/Debug")
ELSE()
    SET(THIRDPARTY_LIBRARIES "D:/ThirdPartyLibraries-20210530/x64/vc15/lib/Release")
ENDIF()

IF(DEBUG_ON)
	MESSAGE(STATUS "[Debug Mode]")
	SET(CMAKE_BUILD_TYPE "Debug") 
	SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
  	SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  	SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
ELSE()
	MESSAGE(STATUS "[Release Mode]")
	SET(CMAKE_BUILD_TYPE "Release") 
	SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
  	SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  	SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
ENDIF()

########################################################################
# third parties
find_package(OpenMP REQUIRED)
IF(OPENMP_FOUND)
  	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF()

SET(EIGEN3_INCLUDE_DIR ${THIRDPARTY_INCLUDE})
#find_package(Eigen3 REQUIRED QUIET)
IF(EIGEN3_FOUND) 
	INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
	MESSAGE("EIGEN3 [OK]:" ${EIGEN3_INCLUDE_DIR})
ENDIF(EIGEN3_FOUND)

SET(OpenCV_INCLUDE_DIRS ${THIRDPARTY_INCLUDE})
#find_package(OpenCV 3.0 REQUIRED QUIET)
IF(OPENCV_FOUND) 
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
	MESSAGE("OPENCV [OK]: " ${OpenCV_INCLUDE_DIRS}) 
	#ADD_DEFINITIONS(-DOPENCV_ON)
ENDIF(OPENCV_FOUND)

SET(Boost_INCLUDE_DIRS ${THIRDPARTY_INCLUDE})
#find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
IF(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	MESSAGE("BOOST [OK]:" ${Boost_INCLUDE_DIRS}) 
ENDIF(Boost_FOUND)

SET(CERES_INCLUDE_DIRS ${THIRDPARTY_INCLUDE})
#find_package(Ceres REQUIRED QUIET)
IF(Ceres_FOUND)
	INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
	MESSAGE("CERES [OK]:" ${CERES_INCLUDE_DIRS}) 
ENDIF(Ceres_FOUND)

########################################################################
# include folder
# third party library include directory for whole sln
INCLUDE_DIRECTORIES(
    ${THIRDPARTY_INCLUDE}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

FILE(GLOB_RECURSE HEADER_FILES_APRILTAG 
	${PROJECT_SOURCE_DIR}/include/camera_model/apriltag/*.h & 
	${PROJECT_SOURCE_DIR}/src/apriltag/*.cc
)
SOURCE_GROUP("apriltag" FILES ${HEADER_FILES_APRILTAG})

FILE(GLOB_RECURSE HEADER_FILES_APRILTAG_FRONTEND 
	${PROJECT_SOURCE_DIR}/include/camera_model/apriltag_frontend/*.hpp & 
	${PROJECT_SOURCE_DIR}/src/apriltag_frontend/*.cpp
)
SOURCE_GROUP("apriltagfrontend" FILES ${HEADER_FILES_APRILTAG_FRONTEND})

FILE(GLOB_RECURSE HEADER_FILES_CALIB
	${PROJECT_SOURCE_DIR}/include/camera_model/calib/*.h & 
	${PROJECT_SOURCE_DIR}/src/calib/*.cc
)
SOURCE_GROUP("calib" FILES ${HEADER_FILES_CALIB})

FILE(GLOB_RECURSE HEADER_FILES_CAMERA_MODELS 
	${PROJECT_SOURCE_DIR}/include/camera_model/camera_models/*.h &
	${PROJECT_SOURCE_DIR}/src/camera_models/*.cc &
	${PROJECT_SOURCE_DIR}/src/camera_models/*.cpp
)
SOURCE_GROUP("cameramodels" FILES ${HEADER_FILES_CAMERA_MODELS})

FILE(GLOB_RECURSE HEADER_FILES_CHESSBOARD 
	${PROJECT_SOURCE_DIR}/include/camera_model/chessboard/*.h &
	${PROJECT_SOURCE_DIR}/src/chessboard/*.cc
)
SOURCE_GROUP("chessboard" FILES ${HEADER_FILES_CHESSBOARD})

FILE(GLOB_RECURSE HEADER_FILES_UTIL 
	${PROJECT_SOURCE_DIR}/include/camera_model/code_utils/*.h &
	${PROJECT_SOURCE_DIR}/include/camera_model/code_utils/math_utils/*.h &
	${PROJECT_SOURCE_DIR}/src/code_utils/*.cc &
	${PROJECT_SOURCE_DIR}/src/code_utils/*.cpp &
	${PROJECT_SOURCE_DIR}/src/code_utils/math_utils/*.cpp
)
SOURCE_GROUP("util" FILES ${HEADER_FILES_UTIL})

FILE(GLOB_RECURSE HEADER_FILES_GPL
	${PROJECT_SOURCE_DIR}/include/camera_model/gpl/*.h & 
	${PROJECT_SOURCE_DIR}/src/gpl/*.cc
)
SOURCE_GROUP("gpl" FILES ${HEADER_FILES_GPL})

FILE(GLOB_RECURSE HEADER_FILES_SPARSE_GRAPH 
	${PROJECT_SOURCE_DIR}/include/camera_model/sparse_graph/*.h & 
	${PROJECT_SOURCE_DIR}/src/sparse_graph/*.cc
)
SOURCE_GROUP("sparsegraph" FILES ${HEADER_FILES_SPARSE_GRAPH})

########################################################################
# library base path
LINK_DIRECTORIES(
	${THIRDPARTY_LIBRARIES}
)

#AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/apriltag AprilTagSrc)
#AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/apriltag_frontend AprilTagSrc2)

# ET(CAMERA_MODEL_FILES
#    ${PROJECT_SOURCE_DIR}/src/camera_models/Camera.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/CostFunctionFactory.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/CameraFactory.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/PinholeCamera.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/PinholeFullCamera.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/CataCamera.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/EquidistantCamera.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/ScaramuzzaCamera.cc
#    ${PROJECT_SOURCE_DIR}/src/camera_models/PolyFisheyeCamera.cpp
#    ${PROJECT_SOURCE_DIR}/src/camera_models/SplineCamera.cpp
#    ${PROJECT_SOURCE_DIR}/src/camera_models/FovCamera.cpp
#    ${PROJECT_SOURCE_DIR}/src/sparse_graph/Transform.cc
#    ${PROJECT_SOURCE_DIR}/src/gpl/gpl.cc
#    ${PROJECT_SOURCE_DIR}/src/code_utils/math_utils/Polynomial.cpp
#    ${PROJECT_SOURCE_DIR}/src/code_utils/cv_utils.cc
#    ${PROJECT_SOURCE_DIR}/src/gpl/EigenQuaternionParameterization.cc
#    ${AprilTagSrc}
#    ${AprilTagSrc2}
#    )

########################################################################
# extern shared library e.g. CameraModel.lib
ADD_EXECUTABLE(${EXE_NAME}
	src/backward.hpp
	src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
	${HEADER_FILES_APRILTAG}
	${HEADER_FILES_APRILTAG_FRONTEND}
	${HEADER_FILES_CALIB}
	${HEADER_FILES_CHESSBOARD}
    ${HEADER_FILES_CAMERA_MODELS}
	${HEADER_FILES_UTIL}
	${HEADER_FILES_GPL}
	${HEADER_FILES_SPARSE_GRAPH}
    )
TARGET_LINK_LIBRARIES(${EXE_NAME}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES} 
	ceres
	gflags
	glog
	opencv_core347
	opencv_calib3d347
	opencv_highgui347
	opencv_imgproc347
	opencv_world347
    )

########################################################################
# othere executable file
# add_executable(Calibration 
#     src/intrinsic_calib.cc
#     src/chessboard/Chessboard.cc
#     src/calib/CameraCalibration.cc
#     ${CAMERA_MODEL_FILES}
#     )
# add_executable(april_calib
#     src/april_calib.cc
#     src/chessboard/Chessboard.cc
#     src/calib/CameraCalibration.cc
#     ${CAMERA_MODEL_FILES}
#     )
#  add_executable(Test
#      src/test_calib.cc
#      src/chessboard/Chessboard.cc
#      src/calib/CameraCalibrationTest.cc
#      ${CAMERA_MODEL_FILES}
#      )
# 
# target_link_libraries(Calibration
#     ${Boost_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${CERES_LIBRARIES}
#     ${EXE_NAME}    
#     )
# target_link_libraries(april_calib
#     ${Boost_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${CERES_LIBRARIES}
#     ${EXE_NAME}    
#     )
# target_link_libraries(Test
#     ${Boost_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${CERES_LIBRARIES}
#     ${EXE_NAME}   
#      )

########################################################################
# C++ PREPROCESSOR DEFINITIONS, carefully note the -D PREFIX
IF(MSVC)
	TARGET_COMPILE_OPTIONS(${EXE_NAME} PUBLIC "-D_USE_MATH_DEFINES")
	TARGET_COMPILE_OPTIONS(${EXE_NAME} PUBLIC "-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
	 ADD_DEFINITIONS("/EHsc") # for error link 2001 __cdecl boost::throw_exception
ENDIF(MSVC)