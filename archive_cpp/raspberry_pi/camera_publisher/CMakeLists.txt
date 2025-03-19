cmake_minimum_required(VERSION 3.10)
project(webcam_publisher)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

# Generate protobuf files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/image.proto)

# Add executable
add_executable(webcam_publisher 
    code/main.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

# Include directories
target_include_directories(webcam_publisher PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${eCAL_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}  # For generated protobuf files
)

# Link libraries
target_link_libraries(webcam_publisher
    PRIVATE
        eCAL::core
        eCAL::proto
        ${OpenCV_LIBS}
        ${PROTOBUF_LIBRARIES}
)