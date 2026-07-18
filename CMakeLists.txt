    #
    # @file     CMakeLists.txt
    # @brief    root CMakeList.txt for sub projects
    # @author   Nico Verduin
    # @date     29-3-2023
    #

    # Add support for printing floating point numbers
    # Please remove if the target becomes too big on a small microcontroller!
    add_link_options(-Wl,-u,vfprintf)
    add_link_options(-lprintf_flt)

    # do not change this last line below
    include(../generic.cmake)

