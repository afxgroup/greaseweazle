cmake_minimum_required(VERSION 3.14)
project(greaseweazle VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find libusb 0.1 (libusb-compat for modern systems)
find_path(LIBUSB_INCLUDE_DIR usb.h
    PATHS /usr/include /usr/local/include
)

find_library(LIBUSB_LIBRARY
    NAMES usb usb-0.1
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

if(NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
    message(FATAL_ERROR "libusb 0.1 not found. Please install libusb-compat-0.1-dev")
endif()

message(STATUS "Found libusb 0.1: ${LIBUSB_LIBRARY}")
message(STATUS "libusb 0.1 include: ${LIBUSB_INCLUDE_DIR}")

# Create header-only library interface
add_library(greaseweazle INTERFACE)
target_include_directories(greaseweazle INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LIBUSB_INCLUDE_DIR}
)
target_link_libraries(greaseweazle INTERFACE ${LIBUSB_LIBRARY})
target_compile_features(greaseweazle INTERFACE cxx_std_17)

# Examples
option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS greaseweazle
    EXPORT greaseweazle-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT greaseweazle-targets
    FILE greaseweazle-targets.cmake
    NAMESPACE greaseweazle::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/greaseweazle
)

# Create package config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/greaseweazle-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_file(cmake/greaseweazle-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/greaseweazle-config.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/greaseweazle-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/greaseweazle-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/greaseweazle
)
