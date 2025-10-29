cmake_minimum_required(VERSION 3.18)

project(SurroundView LANGUAGES CXX CUDA)

# CUDA settings for Jetson Orin Nano with CUDA 12.6
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_CUDA_ARCHITECTURES 87)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fopenmp")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --use_fast_math")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-extended-lambda")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)

# OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

set(PROJ_INCLUDE_DIRS)
set(PROJ_LIBRARIES)

# 3rd party base directory
set(3DP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3dparty/")

# ============ GLM (OpenGL Mathematics) ============
find_package(glm QUIET)
if(glm_FOUND)
    message(STATUS "✓ Using system GLM")
    #get_target_property(GLM_INCLUDE_DIRS glm::glm INTERFACE_INCLUDE_DIRECTORIES)
    #list(APPEND PROJ_INCLUDE_DIRS ${GLM_INCLUDE_DIRS})
    list(APPEND PROJ_INCLUDE_DIRS "/usr/include/glm")

else()
    message(STATUS "⚠ System GLM not found, trying 3dparty")
    set(GLM_INCLUDE_DIR "${3DP_INCLUDE_DIR}/glm")
    if(EXISTS ${GLM_INCLUDE_DIR})
        message(STATUS "✓ Using 3dparty GLM")
        list(APPEND PROJ_INCLUDE_DIRS ${GLM_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "GLM not found! Install: sudo apt install libglm-dev")
    endif()
endif()

# ============ GLFW (Window/Input Library) ============
find_package(glfw3 QUIET)
if(glfw3_FOUND)
    message(STATUS "✓ Using system GLFW")
    list(APPEND PROJ_LIBRARIES glfw)
    #list(APPEND PROJ_LIBRARIES "/usr/include/GLFW")
else()
    message(STATUS "⚠ System GLFW not found, trying 3dparty")
    set(GLFW_LIB "${3DP_INCLUDE_DIR}/lib/libglfw.so.3.1")
    if(EXISTS ${GLFW_LIB})
        message(STATUS "✓ Using 3dparty GLFW: ${GLFW_LIB}")
        list(APPEND PROJ_LIBRARIES ${GLFW_LIB})
    else()
        message(FATAL_ERROR "GLFW not found! Install: sudo apt install libglfw3-dev")
    endif()
endif()

# ============ EGL/GLES (OpenGL ES) ============
find_library(EGL_LIB EGL)
find_library(GLES_LIB2 GLESv2)

if(EGL_LIB AND GLES_LIB2)
    message(STATUS "✓ Using system EGL/GLES")
    list(APPEND PROJ_LIBRARIES ${EGL_LIB} ${GLES_LIB2})
else()
    message(STATUS "⚠ System EGL/GLES not found, trying 3dparty")
    set(EGL_3DP "${3DP_INCLUDE_DIR}/lib/libEGL.so")
    set(GLES_3DP "${3DP_INCLUDE_DIR}/lib/libGLESv2.so")
    
    if(EXISTS ${EGL_3DP} AND EXISTS ${GLES_3DP})
        message(STATUS "✓ Using 3dparty EGL/GLES")
        list(APPEND PROJ_LIBRARIES ${EGL_3DP} ${GLES_3DP})
    else()
        message(FATAL_ERROR "EGL/GLES not found! Install: sudo apt install libgles2-mesa-dev libegl1-mesa-dev")
    endif()
endif()

#========= Find GStreamer====================
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

list(APPEND PROJ_INCLUDE_DIRS ${GSTREAMER_INCLUDE_DIRS})
list(APPEND PROJ_INCLUDE_DIRS ${GSTREAMER_APP_INCLUDE_DIRS})
list(APPEND PROJ_LIBRARIES ${GSTREAMER_LIBRARIES})
list(APPEND PROJ_LIBRARIES ${GSTREAMER_APP_LIBRARIES})

# ============ OpenCV ============
find_package(OpenCV REQUIRED)
message(STATUS "✓ OpenCV ${OpenCV_VERSION} found")
list(APPEND PROJ_LIBRARIES ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})

# ============ CUDA ============
list(APPEND PROJ_LIBRARIES CUDA::cudart CUDA::cuda_driver)
list(APPEND PROJ_INCLUDE_DIRS ${CUDAToolkit_INCLUDE_DIRS})
message(STATUS "✓ CUDA ${CUDAToolkit_VERSION} found")

# ============ OpenGL ============
find_package(OpenGL REQUIRED)
list(APPEND PROJ_INCLUDE_DIRS ${OPENGL_EGL_INCLUDE_DIR})
message(STATUS "✓ OpenGL found")

# ============ ASSIMP (3D Model Loader) ============
find_package(assimp QUIET)
if(NOT assimp_FOUND)
    find_package(ASSIMP QUIET)
endif()

if(assimp_FOUND OR ASSIMP_FOUND)
    message(STATUS "✓ ASSIMP found")
    if(assimp_FOUND)
        list(APPEND PROJ_LIBRARIES assimp::assimp)
    else()
        list(APPEND PROJ_LIBRARIES ${ASSIMP_LIBRARIES})
        list(APPEND PROJ_INCLUDE_DIRS ${ASSIMP_INCLUDE_DIRS})
    endif()
else()
    message(FATAL_ERROR "ASSIMP not found! Install: sudo apt install libassimp-dev")
endif()

# Add 3dparty include directory if it exists
if(EXISTS ${3DP_INCLUDE_DIR})
    list(APPEND PROJ_INCLUDE_DIRS ${3DP_INCLUDE_DIR})
endif()

# Project includes
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files
file(GLOB SOURCE_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cusrc/*.cu
)

# Executable
add_executable(SurroundView ${SOURCE_FILES})

set_target_properties(SurroundView 
    PROPERTIES 
    OUTPUT_NAME "SurroundView" 
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

set_property(TARGET SurroundView PROPERTY CUDA_STANDARD 14)
set_property(TARGET SurroundView PROPERTY CXX_STANDARD 14)

target_link_libraries(SurroundView PUBLIC ${PROJ_LIBRARIES})
target_include_directories(SurroundView PUBLIC ${PROJ_INCLUDE_DIRS})

message(STATUS "=== Configuration Complete ===")
