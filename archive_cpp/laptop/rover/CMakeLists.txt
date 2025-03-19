cmake_minimum_required(VERSION 3.10)
project(person_detection)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

# Generate protobuf files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/image.proto proto/movement.proto)

# Add executable
add_executable(person_detection 
    code/main.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

# Include directories
target_include_directories(person_detection PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${eCAL_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}  # For generated protobuf files
)

# Link libraries
target_link_libraries(person_detection
    PRIVATE
        eCAL::core
        eCAL::proto
        ${OpenCV_LIBS}
        ${PROTOBUF_LIBRARIES}
)