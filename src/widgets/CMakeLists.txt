# Set the SRCS_WIDGET variable and send the result back to the parent CMakeLists files.
set(SRCS_WIDGET
    ${CMAKE_CURRENT_SOURCE_DIR}/arrowhintWidget.cpp    
    ${CMAKE_CURRENT_SOURCE_DIR}/opticalflowWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/oventimerWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pictureflow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/surfaceWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vumeterWidget.cpp
    PARENT_SCOPE)
