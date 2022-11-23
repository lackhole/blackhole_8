cmake_minimum_required(VERSION 3.11)
project(blackhole_8)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(ADDITIONAL_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")
set(ADDITIONAL_LIBRARIES "")

set(BH_RESOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/resource")
set(BH_OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/output")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBH_RESOURCE_DIR_INPUT='${BH_RESOURCE_DIR}'")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBH_OUTPUT_DIR_INPUT='${BH_OUTPUT_DIR}'")

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive-")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:ternary")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2")

    # OpenCV
    find_package(OpenCV REQUIRED)
    list(APPEND ADDITIONAL_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}")
    list(APPEND ADDITIONAL_LIBRARIES "${OpenCV_LIBRARIES}")

    # OpenCL
    set(INTEL_SDK_DIR "C:\\Program Files (x86)\\IntelSWTools\\system_studio_2020")
    set(OPENCL_SDK_DIR "${INTEL_SDK_DIR}\\OpenCL\\sdk")
    set(OpenCL_INCLUDE_DIRS "${OPENCL_SDK_DIR}/include")

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OpenCL_LIBRARIES "${OPENCL_SDK_DIR}/lib/x64/OpenCL.lib")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(OpenCL_LIBRARIES "${OPENCL_SDK_DIR}/lib/x86/OpenCL.lib")
    else()
        message(WARNING "Unable to detect architecture")
    endif()

    if (NOT EXISTS ${OpenCL_LIBRARIES})
        message(WARNING "OpenCL library is not found!")
    else()
        list(APPEND ADDITIONAL_INCLUDE_DIRS "${OpenCL_INCLUDE_DIRS}")
        list(APPEND ADDITIONAL_LIBRARIES "${OpenCL_LIBRARIES}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBH_USE_OPENCL")
    endif()
elseif (EMSCRIPTEN)

elseif (APPLE)
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")

    # OpenCV
    find_package(OpenCV REQUIRED)
    list(APPEND ADDITIONAL_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}")
    list(APPEND ADDITIONAL_LIBRARIES "${OpenCV_LIBRARIES}")

    # OpenCL
    find_package(OpenCL REQUIRED)

    list(APPEND ADDITIONAL_INCLUDE_DIRS "${OpenCL_INCLUDE_DIRS}")
    list(APPEND ADDITIONAL_LIBRARIES "${OpenCL_LIBRARIES}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBH_USE_OPENCL")
endif()

add_executable(blackhole_8 main.cpp)

target_link_libraries(blackhole_8 ${ADDITIONAL_LIBRARIES})
target_include_directories(blackhole_8 PUBLIC ${ADDITIONAL_INCLUDE_DIRS})

enable_testing()

macro(bh_add_test name src)
    add_executable(${name} ${src})
    add_test(NAME ${name} COMMAND ${name})
    target_link_libraries(${name} PUBLIC "${ADDITIONAL_LIBRARIES}")
    target_include_directories(${name} PUBLIC "${ADDITIONAL_INCLUDE_DIRS}")
endmacro()

macro(bh_add_exec name src)
    add_executable(${name} ${src})
    target_link_libraries(${name} PUBLIC "${ADDITIONAL_LIBRARIES}")
    target_include_directories(${name} PUBLIC "${ADDITIONAL_INCLUDE_DIRS}")
endmacro()

bh_add_test(matrix_test include/blackhole/matrix_test.cc)
bh_add_exec(camera_test include/blackhole/camera_test.cc)
bh_add_test(object_test include/blackhole/object/object_test.cc)
bh_add_test(utility_test include/blackhole/utility_test.cc)
bh_add_test(vector_object_test include/blackhole/object/vector_object_test.cc)
bh_add_test(ray_tracer_test include/blackhole/ray_tracer_test.cc)
bh_add_test(blackhole_solution_test include/blackhole/blackhole_solution_test.cc)
#bh_add_test(multiple_blackhole_solution_test include/blackhole/multiple_blackhole_solution_test.cc)
#bh_add_test(multiple_blackhole_solution_test2 include/blackhole/multiple_blackhole_solution_test2.cc)


