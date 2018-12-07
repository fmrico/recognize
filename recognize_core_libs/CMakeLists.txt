cmake_minimum_required(VERSION 2.8.3)
project(recognize_core_libs)

add_compile_options(--std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  roslint
  recognize_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package (Threads)

set(DARKNET_DIR ../ThirdParty/darknet)

catkin_package(
   INCLUDE_DIRS include ${DARKNET_DIR}/src
   LIBRARIES recognize_core_libs darknet
   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${DARKNET_DIR}/src
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DOPENCV)

if (CUDA)
  FIND_PACKAGE(CUDA)
endif()

if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()

  list(APPEND LIBRARIES "m")
endif()

set(DARKNET_SRCS
  ${DARKNET_DIR}/src/activation_layer.c
  ${DARKNET_DIR}/src/activations.c
  ${DARKNET_DIR}/src/art.c
  ${DARKNET_DIR}/src/avgpool_layer.c
  ${DARKNET_DIR}/src/batchnorm_layer.c
  ${DARKNET_DIR}/src/blas.c
  ${DARKNET_DIR}/src/box.c
  ${DARKNET_DIR}/src/captcha.c
  ${DARKNET_DIR}/src/cifar.c
  ${DARKNET_DIR}/src/yolo_v2_class.cpp
  ${DARKNET_DIR}/src/classifier.c
  ${DARKNET_DIR}/src/coco.c
  ${DARKNET_DIR}/src/col2im.c
  ${DARKNET_DIR}/src/compare.c
  ${DARKNET_DIR}/src/connected_layer.c
  ${DARKNET_DIR}/src/convolutional_layer.c
  ${DARKNET_DIR}/src/cost_layer.c
  ${DARKNET_DIR}/src/cpu_gemm.c
  ${DARKNET_DIR}/src/crnn_layer.c
  ${DARKNET_DIR}/src/crop_layer.c
  ${DARKNET_DIR}/src/cuda.c
  ${DARKNET_DIR}/src/darknet.c
  ${DARKNET_DIR}/src/data.c
  ${DARKNET_DIR}/src/deconvolutional_layer.c
  ${DARKNET_DIR}/src/demo.c
  ${DARKNET_DIR}/src/detection_layer.c
  ${DARKNET_DIR}/src/detector.c
  ${DARKNET_DIR}/src/dice.c
  ${DARKNET_DIR}/src/dropout_layer.c
  ${DARKNET_DIR}/src/gemm.c
  ${DARKNET_DIR}/src/getopt.c
  ${DARKNET_DIR}/src/go.c
  ${DARKNET_DIR}/src/gru_layer.c
  ${DARKNET_DIR}/src/http_stream.cpp
  ${DARKNET_DIR}/src/im2col.c
  ${DARKNET_DIR}/src/image.c
  ${DARKNET_DIR}/src/layer.c
  ${DARKNET_DIR}/src/list.c
  ${DARKNET_DIR}/src/local_layer.c
  ${DARKNET_DIR}/src/matrix.c
  ${DARKNET_DIR}/src/maxpool_layer.c
  ${DARKNET_DIR}/src/network.c
  ${DARKNET_DIR}/src/nightmare.c
  ${DARKNET_DIR}/src/normalization_layer.c
  ${DARKNET_DIR}/src/option_list.c
  ${DARKNET_DIR}/src/parser.c
  ${DARKNET_DIR}/src/region_layer.c
  ${DARKNET_DIR}/src/reorg_layer.c
  ${DARKNET_DIR}/src/reorg_old_layer.c
  ${DARKNET_DIR}/src/rnn.c
  ${DARKNET_DIR}/src/rnn_layer.c
  ${DARKNET_DIR}/src/rnn_vid.c
  ${DARKNET_DIR}/src/route_layer.c
  ${DARKNET_DIR}/src/shortcut_layer.c
  ${DARKNET_DIR}/src/softmax_layer.c
  ${DARKNET_DIR}/src/super.c
  ${DARKNET_DIR}/src/swag.c
  ${DARKNET_DIR}/src/tag.c
  ${DARKNET_DIR}/src/tree.c
  ${DARKNET_DIR}/src/upsample_layer.c
  ${DARKNET_DIR}/src/utils.c
  ${DARKNET_DIR}/src/voxel.c
  ${DARKNET_DIR}/src/writing.c
  ${DARKNET_DIR}/src/yolo.c
  ${DARKNET_DIR}/src/yolo_layer.c
)
if (CUDA_FOUND)
  set(DARKNET_SRCS
    ${DARKNET_SRCS}
    ${DARKNET_DIR}/src/activation_kernels.cu
    ${DARKNET_DIR}/src/avgpool_layer_kernels.cu
    ${DARKNET_DIR}/src/blas_kernels.cu
    ${DARKNET_DIR}/src/col2im_kernels.cu
    ${DARKNET_DIR}/src/convolutional_kernels.cu
    ${DARKNET_DIR}/src/crop_layer_kernels.cu
    ${DARKNET_DIR}/src/deconvolutional_kernels.cu
    ${DARKNET_DIR}/src/dropout_layer_kernels.cu
    ${DARKNET_DIR}/src/im2col_kernels.cu
    ${DARKNET_DIR}/src/maxpool_layer_kernels.cu
    ${DARKNET_DIR}/src/network_kernels.cu
  )
  cuda_add_library(darknet SHARED
    ${DARKNET_SRCS}
  )
  target_link_libraries(darknet
    cuda
    cudart
    cublas
    curand
    ${OpenCV_LIBRARIES}
  )
else()
  add_library(darknet SHARED
    ${DARKNET_SRCS}
  )
endif()

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/utils.cpp
   src/${PROJECT_NAME}/Labels.cpp
   src/${PROJECT_NAME}/translations.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME} darknet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ${DARKNET_DIR}/src
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

roslint_cpp(
  src/${PROJECT_NAME}/utils.cpp include/${PROJECT_NAME}/utils.h
  src/${PROJECT_NAME}/Labels.cpp include/${PROJECT_NAME}/Labels.h
  test/test_darknet.cpp
  test/test_Labels.cpp
  test/test_utils.cpp
)

catkin_add_gtest(test_darknet test/test_darknet.cpp)
if(TARGET test_darknet)
   target_link_libraries(test_darknet ${PROJECT_NAME} ${OpenCV_LIBS} darknet ${catkin_LIBRARIES} gtest)
endif()
catkin_add_gtest(test_Labels test/test_Labels.cpp)
if(TARGET test_Labels)
   target_link_libraries(test_Labels ${PROJECT_NAME} ${OpenCV_LIBS} darknet gtest ${catkin_LIBRARIES} gtest)
endif()
catkin_add_gtest(test_utils test/test_utils.cpp)
if(TARGET test_utils)
   target_link_libraries(test_utils ${PROJECT_NAME} ${OpenCV_LIBS} darknet ${catkin_LIBRARIES} gtest)
endif()
