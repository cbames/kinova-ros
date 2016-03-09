cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0015 NEW)

project(kinova_robot)

include_directories(
    include/
)
message("${PROJECT_SOURCE_DIR}/lib")

find_library(KINOVA_CMD_LIBRARY
    kinova_cmd
    HINTS "${PROJECT_SOURCE_DIR}/lib"
)

find_library(KINOVA_COMM_LIBRARY
    kinova_comm 
    HINTS "${PROJECT_SOURCE_DIR}/lib"
) 


#message(${KINOVA_CMD_LIBRARY})
#message(${KINOVA_COMM_LIBRARY})

add_library(${PROJECT_NAME} SHARED
    src/kinova_robot.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${KINOVA_CMD_LIBRARY}
    ${KINOVA_COMM_LIBRARY}
)
