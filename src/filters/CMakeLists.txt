
set(SRCS_FILTER
    ${CMAKE_CURRENT_SOURCE_DIR}/filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/alphabetFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/calibrationFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/debounceFilter.h # exists as a template
    ${CMAKE_CURRENT_SOURCE_DIR}/delayFilter.h # exists as a template
    ${CMAKE_CURRENT_SOURCE_DIR}/doubleClickFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fingertipFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joystickFilter.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/gestureFilter.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/letterFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/meanFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/objectDetectionFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/otsuFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pidFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/quadrantFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rotationFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/swapFilter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/switchFilter.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/timeAverageFilter.cpp
    PARENT_SCOPE)
