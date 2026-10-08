cmake_minimum_required(VERSION 3.22)

project(ThrustAllocator VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(THRUST_ALLOCATOR_BUILD_EXAMPLE "Build bundled example" ON)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party")
set(EIGEN_DIR "${THIRD_PARTY_DIR}/Eigen")
set(OSQP_DIR "${THIRD_PARTY_DIR}/osqp")
set(QDLDL_DIR "${THIRD_PARTY_DIR}/qdldl")
set(OSQP_EIGEN_DIR "${THIRD_PARTY_DIR}/osqp-eigen")

foreach(required_path
    "${EIGEN_DIR}/Eigen/Core"
    "${OSQP_DIR}/CMakeLists.txt"
    "${QDLDL_DIR}/CMakeLists.txt"
    "${OSQP_EIGEN_DIR}/include/OsqpEigen/OsqpEigen.h")
    if(NOT EXISTS "${required_path}")
        message(FATAL_ERROR "Missing bundled dependency: ${required_path}")
    endif()
endforeach()

add_library(eigen_headers INTERFACE)
target_include_directories(eigen_headers INTERFACE "${EIGEN_DIR}")
add_library(Eigen3::Eigen ALIAS eigen_headers)

set(FETCHCONTENT_SOURCE_DIR_QDLDL "${QDLDL_DIR}" CACHE PATH "Use bundled QDLDL" FORCE)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "" FORCE)
set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PROFILING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)

add_subdirectory(
    "${OSQP_DIR}"
    "${CMAKE_BINARY_DIR}/third_party/osqp"
    EXCLUDE_FROM_ALL
)

if(NOT TARGET OSQPLIB)
    message(FATAL_ERROR "OSQP target OSQPLIB was not created")
endif()
if(NOT TARGET qdldlobject)
    message(FATAL_ERROR "OSQP target qdldlobject was not created")
endif()

add_library(
    thrust_allocator STATIC
    src/ThrustAllocationCore.cpp
    src/thrust_allocator_c.cpp
    "${OSQP_EIGEN_DIR}/src/Data.cpp"
    "${OSQP_EIGEN_DIR}/src/Settings.cpp"
    "${OSQP_EIGEN_DIR}/src/Solver.cpp"
    "${OSQP_EIGEN_DIR}/src/Debug.cpp"
    "${OSQP_DIR}/src/osqp_api.c"
    $<TARGET_OBJECTS:OSQPLIB>
    $<TARGET_OBJECTS:qdldlobject>
)
add_library(ThrustAllocator::thrust_allocator ALIAS thrust_allocator)

get_target_property(OSQP_INTERNAL_INCLUDE_DIRS OSQPLIB INCLUDE_DIRECTORIES)
if(NOT OSQP_INTERNAL_INCLUDE_DIRS)
    message(FATAL_ERROR "Could not obtain OSQPLIB include directories")
endif()

target_include_directories(
    thrust_allocator
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${EIGEN_DIR}"
        "${OSQP_EIGEN_DIR}/include"
        ${OSQP_INTERNAL_INCLUDE_DIRS}
)

target_compile_definitions(
    thrust_allocator
    PRIVATE
        OSQP_EIGEN_OSQP_IS_V1
        OSQP_EIGEN_OSQP_IS_V1_FINAL
)

target_compile_features(thrust_allocator PUBLIC cxx_std_17)
set_target_properties(
    thrust_allocator
    PROPERTIES
        OUTPUT_NAME "thrust_allocator"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

if(THRUST_ALLOCATOR_BUILD_EXAMPLE)
    add_executable(thrust_allocator_example example/example.cpp)
    target_link_libraries(
        thrust_allocator_example
        PRIVATE ThrustAllocator::thrust_allocator
    )
    set_target_properties(
        thrust_allocator_example
        PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
    TARGETS thrust_allocator
    EXPORT ThrustAllocatorTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ThrustAllocatorTargets
    FILE ThrustAllocatorTargets.cmake
    NAMESPACE ThrustAllocator::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ThrustAllocator
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ThrustAllocatorConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ThrustAllocatorConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ThrustAllocator
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ThrustAllocatorConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ThrustAllocatorConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ThrustAllocatorConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ThrustAllocator
)
