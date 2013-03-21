# ======================================================
# MagicPad
# ======================================================
cmake_minimum_required(VERSION 2.8.7)

# Project name
project(MAGICPAD)

# include cmake scripts
include(CMake/magicpad-macros.cmake)
include(CMake/magicpad-testing.cmake)
include(CMake/magicpad-version.cmake)

#
# DLLs'
#
get_filename_component(MINGW_DLL_DIRECTORY ${CMAKE_CXX_COMPILER} PATH)
magicpad_install_files(${MINGW_DLL_DIRECTORY}/libgcc_s_dw2-1.dll ".")
magicpad_install_files(${MINGW_DLL_DIRECTORY}/libstdc++-6.dll ".")
magicpad_install_files(${MINGW_DLL_DIRECTORY}/mingwm10.dll ".")

#
# Find QT
#
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
set(QT_USE_QTWEBKIT TRUE)
set(QT_USE_PHONON TRUE)

include(${QT_USE_FILE})

set(CMAKE_AUTOMOC TRUE)

set(QtApp_RCCS resources.qrc)
qt4_add_resources(QtApp_RCC_SRCS ${QtApp_RCCS})

get_filename_component(QT_DLL_PATH ${QT_QMAKE_EXECUTABLE} PATH)
magicpad_install_files(${QT_DLL_PATH}/QtCore4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/QtGui4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/QtOpenGL4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/QtXml4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/QtWebkit4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/QtNetwork4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/../plugins/imageformats/qjpeg4.dll "imageformats/.")
magicpad_install_files(${QT_DLL_PATH}/phonon4.dll ".")
magicpad_install_files(${QT_DLL_PATH}/../plugins/phonon_backend/phonon_ds94.dll "phonon_backend/.")

#
# Qwt3D
#
magicpad_install_files(3rdparty/qwt3d/lib/qwtplot3d.dll ".")
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/qwt3d/include)
link_directories(${PROJECT_SOURCE_DIR}/3rdparty/qwt3d/lib)

#
# Find OpenGL
#
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

#
# Find FTDI library
#
include_directories("${PROJECT_SOURCE_DIR}/3rdparty/ftdi")
link_directories("${PROJECT_SOURCE_DIR}/3rdparty/ftdi/i386")
magicpad_install_files("${PROJECT_SOURCE_DIR}/3rdparty/ftdi/i386/ftd2xx.dll" ".")

#
# QsLog library
#
include_directories("3rdparty/QsLog")
add_subdirectory("3rdparty/QsLog")

#
# Find OpenCV & GLUT
#
find_package(OpenCV REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
if(NOT(OpenCV_VERSION STREQUAL "2.4.0"))
  message("Your OpenCV version : " ${OpenCV_VERSION})
  message("The requiered version : 2.4.0")
endif(NOT(OpenCV_VERSION STREQUAL "2.4.0"))

#
# Plugins
#
magicpad_install_files("${PROJECT_SOURCE_DIR}/data/.placeholder" "plugins/.")
set(PLUGIN_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/plugins")

add_subdirectory(src)
include_directories(src)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-enable-auto-import")

#set(CMAKE_EXE_LINKER_FLAGS "-static")
add_executable(MagicPad WIN32
    ${MAGICPAD_SRCS}
    ${QSLOG_SRCS}
    ${ENUMSER_SRCS}
    ${QtApp_RCC_SRCS}    
    icon.rc
    )

target_link_libraries(MagicPad
    ${QT_LIBRARIES}
    qwtplot3d
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES} #${GLUT_LIBRARY}
    ftd2xx
    )

install(TARGETS MagicPad DESTINATION ".")

#
# Copy files
#
magicpad_install_files("config.ini" ".")

#
# Install all files in the 'data' subdirectory
#
file(GLOB_RECURSE DATA_FILES "data/*")
foreach(dfile ${DATA_FILES})
    file(RELATIVE_PATH  drpath "${PROJECT_SOURCE_DIR}" ${dfile})
    get_filename_component(subpath ${drpath} PATH)
    magicpad_install_files(${dfile} "${subpath}/.")
endforeach()

magicpad_install_files("Presentation ISORG.pdf"  ".")

#
# Copy drivers
#
file(GLOB_RECURSE DRIVER_FILES "drivers/*")
foreach(dfile ${DRIVER_FILES})
    file(RELATIVE_PATH  drpath "${PROJECT_SOURCE_DIR}" ${dfile})
    get_filename_component(subpath ${drpath} PATH)
    magicpad_install_files(${dfile} "${subpath}/.")
endforeach()

#
# Documentation
#
#add_subdirectory(doc)

#
# Installer
#
include(CMake/magicpad-installer.cmake)
