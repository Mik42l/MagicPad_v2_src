###############################################################################
#
# PluginExample CMakeLists.txt
#
# TODO: instruction on how to add some files
#
###############################################################################

cmake_minimum_required(VERSION 2.8.7)

# This is the name that shows up as the project name is Qt Creator
project(PLUGIN_EXAMPLEPLUGIN)

# output dll base name
set(PLUGIN_NAME "pluginexample")

#
# Find opencv
#
find_package(OpenCV REQUIRED)

#
# Find Qt
#
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
set(CMAKE_AUTOMOC TRUE)
set(QtApp_RCCS resources.qrc)
qt4_add_resources(QtApp_RCC_SRCS ${QtApp_RCCS})

#
# Find MagicPad-src
#
if(NOT DEFINED MAGICPAD_SOURCE_DIR)
	set(MAGICPAD_SOURCE_DIR "" CACHE PATH "Should point to the source directory of MagicPad.")
	message(FATAL_ERROR "MAGICPAD_SOURCE_DIR variable should point to the source directory of MagicPad (where the main CMakeLists.txt is)")
else(NOT DEFINED MAGICPAD_SOURCE_DIR)
	if(EXISTS "${MAGICPAD_SOURCE_DIR}/src/applets/appletInterface.h")
		include_directories(${MAGICPAD_SOURCE_DIR}/src)
	else(EXISTS "${MAGICPAD_SOURCE_DIR}/src/applets/appletInterface.h")
		message(FATAL_ERROR "MAGICPAD_SOURCE_DIR is not valid. MAGICPAD_SOURCE_DIR variable should point to the source directory of MagicPad (where the main CMakeLists.txt is)")
	endif(EXISTS "${MAGICPAD_SOURCE_DIR}/src/applets/appletInterface.h")
endif(NOT DEFINED MAGICPAD_SOURCE_DIR)

#
# Find MagicPad installation directory
#
#set(MAGICPAD_INSTALL_DIR CACHE PATH)
if(NOT DEFINED MAGICPAD_INSTALL_DIR)
	set(MAGICPAD_INSTALL_DIR "" CACHE PATH "Should point to the installation direcory of MagicPad (where MagicPad.exe is)")
	message(FATAL_ERROR "MAGICPAD_INSTALL_DIR variable should point to the installation direcory of MagicPad (where MagicPad.exe is)")
else(NOT DEFINED MAGICPAD_INSTALL_DIR)
	set(PLUGIN_DIRECTORY ${MAGICPAD_INSTALL_DIR}/plugins)
endif(NOT DEFINED MAGICPAD_INSTALL_DIR)
message(STATUS "Plugin output directory is:" ${PLUGIN_DIRECTORY})

#
# Build plugin DLL
#
add_library(${PLUGIN_NAME} SHARED 
	${CMAKE_CURRENT_SOURCE_DIR}/examplepluginApplet.cpp
	${QtApp_RCC_SRCS}
	)	
	
target_link_libraries(${PLUGIN_NAME} ${QT_LIBRARIES} ${OpenCV_LIBS})

set_target_properties(${PLUGIN_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PLUGIN_DIRECTORY})
