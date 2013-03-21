#
# Project sources
#
add_subdirectory(applets)
add_subdirectory(com)
add_subdirectory(core)
add_subdirectory(filters)
add_subdirectory(widgets)

set(MAGICPAD_SRCS
    ${SRCS_APPLET}
    ${SRCS_COM}
    ${SRCS_CORE}
    ${SRCS_FILTER}
    ${SRCS_WIDGET}
    PARENT_SCOPE)
