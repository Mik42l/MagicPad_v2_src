
set(SRCS_CORE
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/actionButton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/appletButton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loggerDialog.cpp
    PARENT_SCOPE)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/version.h.cmake" "${CMAKE_CURRENT_SOURCE_DIR}/version.h" @ONLY)
