# CPack configuration for NSIS installer
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_NAME                  "MagicPad")

set(CPACK_PACKAGE_VERSION_MAJOR         ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR         ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH         ${PROJECT_VERSION_PATCH})

set(CPACK_PACKAGE_ICON                  "${PROJECT_SOURCE_DIR}\\\\image\\\\icon.ico")
set(CPACK_PACKAGE_VENDOR                "Isorg, www.isorg.fr")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY   "MagicPad Demonstration Software")
set(CPACK_PACKAGE_DESCRIPTION_FILE      "${CMAKE_CURRENT_SOURCE_DIR}/DESCRIPTION")

set(CPACK_RESOURCE_FILE_README          "${CMAKE_CURRENT_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_WELCOME         "${CMAKE_CURRENT_SOURCE_DIR}/WELCOME")
set(CPACK_RESOURCE_FILE_LICENSE         "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

set(CPACK_PACKAGE_EXECUTABLES           "..\\\\MagicPad ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}\\\\MagicPad" "MagicPad ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

set(CPACK_SET_DESTDIR                   true)
set(CPACK_INSTALL_PREFIX                "MagicPad ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY     "ISORG")

# There is a bug in NSI that does not handle full unix paths properly. Make
# sure there is at least one set of four (4) backlasshes.
set(CPACK_NSIS_INSTALLED_ICON_NAME      "${PROJECT_SOURCE_DIR}\\\\image\\\\icon.ico")
set(CPACK_NSIS_DISPLAY_NAME             "MagicPad ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

# TODO: provide link to a wiki or github
set(CPACK_NSIS_HELP_LINK                "http://www.isorg.fr")
# TODO: provide link to a more thorough description of what the MagicPad can do.
set(CPACK_NSIS_URL_INFO_ABOUT           "http://www.isorg.fr")
set(CPACK_NSIS_CONTACT                  "adrien.bailly@isorg.fr")

include(CPack)
