# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/hw3)
ADD_EXECUTABLE (hw3 hw3.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw3 simviz_hw3.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw3_slow simviz_hw3_slow.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw3_force simviz_hw3_force.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw3_force_slow simviz_hw3_force_slow.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw3 ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw3 ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw3_slow ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw3_force ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw3_force_slow ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY models/world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/world_w_box.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY models/panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
