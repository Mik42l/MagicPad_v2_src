
#
# Usage:
# set(files foo.h bar/bar.h)
# set(dir include/fooheaders)
# magicpad_install_files("${headers}" ${headers_location})
#
function(magicpad_install_files files dir)
    foreach(f ${files})
        get_filename_component(file_dir "${f}" PATH)
        get_filename_component(file_name "${f}" NAME)
        #message(STATUS ${file_dir} " # " ${file_name})
        configure_file("${f}" "${CMAKE_CURRENT_BINARY_DIR}/${dir}" COPYONLY)
        install(FILES "${f}" DESTINATION "${dir}")
    endforeach()
endfunction()
