cmake_minimum_required(VERSION 3.8)
project(analogx_interface_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/Analogx1.msg"
	"msg/BrakeTemp.msg"
	"msg/FrontLeftExternalTireTemp.msg"
	"msg/FrontRightExternalTireTemp.msg"
	"msg/RearLeftExternalTireTemp.msg"
	"msg/RearRightExternalTireTemp.msg"
	"msg/BrakeSensorBodyTemp.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
        std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()