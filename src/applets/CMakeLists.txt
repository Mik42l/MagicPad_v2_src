
file(GLOB PONG_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/pong/*.cpp")

set(SRCS_APPLET
    "${CMAKE_CURRENT_SOURCE_DIR}/alphabetApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/appletInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/applet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/backlightApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bargraphApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bulbApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/djApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gestureApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/imagedisplayApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mapsApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/musicApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/opdApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pandaApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pictureflowApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pongApplet.cpp" ${PONG_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/purpleApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/rollingballApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/slideshowApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/surfaceApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/switchApplet.cpp"
    #"${CMAKE_CURRENT_SOURCE_DIR}/templateApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/twistApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tyuuApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/vumeterApplet.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/newprojectApplet.cpp"
    PARENT_SCOPE)

# build test plugin
#set(SRCS_PLUGIN "${CMAKE_CURRENT_SOURCE_DIR}/pluginApplet.cpp" PARENT_SCOPE)

#add_library(pluginapplet SHARED "${CMAKE_CURRENT_SOURCE_DIR}/pluginApplet.cpp")
#target_link_libraries(pluginapplet ${QT_LIBRARIES} ${OpenCV_LIBS})
#SET_TARGET_PROPERTIES(pluginapplet PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PLUGIN_DIRECTORY})
