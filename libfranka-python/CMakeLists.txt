cmake_minimum_required(VERSION 3.10)
project(franka_python_bindings)

# Force use of system compiler instead of conda's
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Set C++17 as required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set C++17 as required
set(CMAKE_BUILD_TYPE Release)

# Enable RPATH handling
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add all necessary library paths to RPATH
if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:$ENV{CONDA_PREFIX}/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/lib")
else()
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/lib")
endif()

# Define BUILDING_DLL for proper symbol export
if(WIN32)
    add_definitions(-DBUILDING_DLL)
endif()

# Find Python through conda environment if specified, otherwise use system Python
if(DEFINED ENV{CONDA_PREFIX})
    set(Python3_ROOT_DIR $ENV{CONDA_PREFIX})
    set(Python3_EXECUTABLE ${Python3_ROOT_DIR}/bin/python3)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    
    # Add conda include directories
    include_directories(SYSTEM
        $ENV{CONDA_PREFIX}/include
        $ENV{CONDA_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/numpy/core/include
    )

    # Add conda library directory to find fmt
    list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
else()
    # Use system Python
    find_package(Python3 3.8 COMPONENTS Interpreter Development REQUIRED)
endif()

# Set pybind11 policy to avoid warnings
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
endif()

# Find pybind11
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(PYBIND11_CMAKE_DIR)
    set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
    find_package(pybind11 CONFIG REQUIRED)
else()
    find_package(pybind11 REQUIRED)
endif()

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find fmt library (required by libfranka)
find_package(fmt REQUIRED)
message(STATUS "Found fmt: ${fmt_VERSION}")
message(STATUS "fmt library: ${fmt_LIBRARIES}")
message(STATUS "fmt include dir: ${fmt_INCLUDE_DIRS}")

# Find libfranka
if(APPLE)
    # On macOS, try to find libfranka in common locations
    find_package(Franka HINTS 
        /usr/local/lib/cmake/Franka
        /opt/homebrew/lib/cmake/Franka
        /opt/local/lib/cmake/Franka
    )
else()
    # On Linux, try to find libfranka in system locations
    find_package(Franka HINTS
        /usr/lib/cmake/Franka
        /usr/local/lib/cmake/Franka
    )
endif()

if(NOT Franka_FOUND)
    message(FATAL_ERROR 
        "Could not find libfranka. Please install libfranka:\n"
        "Or specify its location with -DFranka_DIR=<path>"
    )
endif()

# Get libfranka's directory
get_target_property(FRANKA_LIBRARY Franka::Franka LOCATION)
get_filename_component(FRANKA_LIB_DIR ${FRANKA_LIBRARY} DIRECTORY)
message(STATUS "Franka library directory: ${FRANKA_LIB_DIR}")

# Add the Python module
pybind11_add_module(_franka_bindings 
    src/franka_bindings.cpp
    src/realtime_control.cpp
)

# Link against libraries
target_link_libraries(_franka_bindings PRIVATE 
    Franka::Franka
    Eigen3::Eigen
    fmt::fmt
)

# Set include directories
target_include_directories(_franka_bindings SYSTEM PRIVATE 
    /usr/include
    /usr/local/include
)

target_include_directories(_franka_bindings PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/realtime_control
    ${Franka_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Add compile definitions to help find headers
target_compile_definitions(_franka_bindings PRIVATE
    FRANKA_INCLUDE_DIR="${FRANKA_ROBOT_INCLUDE_DIR}"
)

# Print include directories for debugging
get_target_property(INCLUDE_DIRS _franka_bindings INCLUDE_DIRECTORIES)
message(STATUS "Include directories: ${INCLUDE_DIRS}")

# Print current compiler info
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C Flags: ${CMAKE_C_FLAGS}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

# Set output directory and properties
set_target_properties(_franka_bindings PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/franka_bindings
    OUTPUT_NAME franka_bindings
    SKIP_BUILD_RPATH FALSE
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(DEFINED ENV{CONDA_PREFIX})
    set_property(TARGET _franka_bindings APPEND PROPERTY 
        INSTALL_RPATH "${FRANKA_LIB_DIR}:$ENV{CONDA_PREFIX}/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/lib"
    )
else()
    set_property(TARGET _franka_bindings APPEND PROPERTY 
        INSTALL_RPATH "${FRANKA_LIB_DIR}:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/lib"
    )
endif()

# Platform-specific settings
if(APPLE)
    set_target_properties(_franka_bindings PROPERTIES 
        CXX_VISIBILITY_PRESET default
        VISIBILITY_INLINES_HIDDEN OFF
        MACOSX_RPATH ON
    )
    target_compile_options(_franka_bindings PRIVATE
        -fvisibility=default
    )
endif()

# Installation rules
include(GNUInstallDirs)
install(TARGETS _franka_bindings
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/franka_bindings
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/franka_bindings
) 