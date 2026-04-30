cmake_minimum_required(VERSION 3.20)

project(XPlaneTruthCapture VERSION 0.1.4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(XPLANE_SDK_ROOT "" CACHE PATH "Path to the X-Plane SDK root directory")
if(NOT XPLANE_SDK_ROOT)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../px4xplane/lib/SDK/CHeaders/XPLM/XPLMPlugin.h")
        set(XPLANE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../px4xplane/lib/SDK" CACHE PATH "Path to the X-Plane SDK root directory" FORCE)
    elseif(DEFINED ENV{XPLANE_SDK_ROOT})
        set(XPLANE_SDK_ROOT "$ENV{XPLANE_SDK_ROOT}" CACHE PATH "Path to the X-Plane SDK root directory" FORCE)
    endif()
endif()

if(NOT EXISTS "${XPLANE_SDK_ROOT}/CHeaders/XPLM/XPLMPlugin.h")
    message(FATAL_ERROR "X-Plane SDK not found. Set -DXPLANE_SDK_ROOT=/path/to/SDK")
endif()

if(WIN32)
    set(XTC_PLATFORM "win")
elseif(APPLE)
    set(XTC_PLATFORM "mac")
else()
    set(XTC_PLATFORM "lin")
endif()

set(XTC_PLUGIN_ROOT "${CMAKE_BINARY_DIR}/XPlaneTruthCapture")
set(XTC_PLUGIN_64 "${XTC_PLUGIN_ROOT}/64")

add_library(XPlaneTruthCapture MODULE
    src/Plugin.cpp
)

target_include_directories(XPlaneTruthCapture PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${XPLANE_SDK_ROOT}/CHeaders/XPLM"
)

target_compile_definitions(XPlaneTruthCapture PRIVATE
    XPLM200=1
    XPLM210=1
    XPLM300=1
    XPLM301=1
    XPLM303=1
)

if(WIN32)
    target_compile_definitions(XPlaneTruthCapture PRIVATE IBM=1 WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS)
    target_link_libraries(XPlaneTruthCapture PRIVATE "${XPLANE_SDK_ROOT}/Libraries/Win/XPLM_64.lib")
elseif(APPLE)
    target_compile_definitions(XPlaneTruthCapture PRIVATE APL=1)
    target_link_options(XPlaneTruthCapture PRIVATE -flat_namespace -undefined dynamic_lookup)
    set_target_properties(XPlaneTruthCapture PROPERTIES OSX_ARCHITECTURES "x86_64;arm64")
else()
    target_compile_definitions(XPlaneTruthCapture PRIVATE LIN=1)
    target_link_options(XPlaneTruthCapture PRIVATE -Wl,--allow-shlib-undefined)
endif()

target_compile_options(XPlaneTruthCapture PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -fvisibility=hidden>
)

set_target_properties(XPlaneTruthCapture PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${XTC_PLATFORM}"
    SUFFIX ".xpl"
    LIBRARY_OUTPUT_DIRECTORY "${XTC_PLUGIN_64}"
    RUNTIME_OUTPUT_DIRECTORY "${XTC_PLUGIN_64}"
)

foreach(CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIG}" CONFIG_UPPER)
    set_target_properties(XPlaneTruthCapture PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${XTC_PLUGIN_64}"
        RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${XTC_PLUGIN_64}"
    )
endforeach()

add_custom_command(TARGET XPlaneTruthCapture POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${XTC_PLUGIN_ROOT}/config"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${XTC_PLUGIN_ROOT}/tools"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/README.md" "${XTC_PLUGIN_ROOT}/README.md"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/TEST_PROTOCOL.md" "${XTC_PLUGIN_ROOT}/TEST_PROTOCOL.md"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/config/capture_config.ini" "${XTC_PLUGIN_ROOT}/config/capture_config.ini"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/config/default_datarefs.txt" "${XTC_PLUGIN_ROOT}/config/default_datarefs.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/config/datarefs.txt" "${XTC_PLUGIN_ROOT}/config/datarefs.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/tools/viewer.html" "${XTC_PLUGIN_ROOT}/tools/viewer.html"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/tools/analyze_capture.py" "${XTC_PLUGIN_ROOT}/tools/analyze_capture.py"
    COMMENT "Staging XPlaneTruthCapture plugin bundle"
)

install(DIRECTORY "${XTC_PLUGIN_ROOT}" DESTINATION ".")

message(STATUS "XPlaneTruthCapture ${PROJECT_VERSION}")
message(STATUS "X-Plane SDK: ${XPLANE_SDK_ROOT}")
message(STATUS "Platform output: ${XTC_PLUGIN_64}/${XTC_PLATFORM}.xpl")
