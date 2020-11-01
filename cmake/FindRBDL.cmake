
find_path(RBDL_INCLUDE_DIR rbdl/rbdl.h
    HINTS
        $ENV{RBDL_INCLUDE_PATH}
        $ENV{RBDL_PATH}/include
        /usr/local/include
        /usr/include
)

find_library(RBDL_LIBRARY NAMES rbdl
    PATHS
        $ENV{RBDL_LIBRARY_PATH}
        $ENV{RBDL_PATH}/lib
        /usr/local/lib
        /usr/lib
)

find_path(RBDL_URDFReader_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
    HINTS
        $ENV{RBDL_INCLUDE_PATH}
        $ENV{RBDL_PATH}/include
        /usr/local/include
        /usr/include
)

find_library(RBDL_URDFReader_LIBRARY NAMES rbdl_urdfreader
    PATHS
        $ENV{RBDL_LIBRARY_PATH}
        $ENV{RBDL_PATH}/lib
        /usr/local/lib
        /usr/lib
)

if(NOT RBDL_LIBRARY)
    message(SEND_ERROR "Not found RBDL library")
endif()

if(NOT RBDL_URDFReader_LIBRARY)
    message(STATUS "Not found RBDL URDFReader library")
endif()

if(RBDL_LIBRARY AND RBDL_INCLUDE_DIR)
    set(RBDL_FOUND TRUE)
endif()

if(RBDL_URDFReader_LIBRARY AND RBDL_URDFReader_INCLUDE_DIR)
    set(RBDL_URDFReader_FOUND TRUE)
endif()

set(RBDL_INCLUDE_DIRS ${RBDL_INCLUDE_DIR} ${RBDL_URDFReader_INCLUDE_DIR})
set(RBDL_LIBRARIES ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})

mark_as_advanced(
    RBDL_INCLUDE_DIR
    RBDL_LIBRARY
    RBDL_URDFReader_INCLUDE_DIR
    RBDL_URDFReader_LIBRARY
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(RBDL
    REQUIRED_VARS
        RBDL_INCLUDE_DIRS
        RBDL_LIBRARIES
)

if(RBDL_FOUND AND NOT TARGET RBDL::RBDL)
    add_library(RBDL::RBDL UNKNOWN IMPORTED)
    set_target_properties(RBDL::RBDL PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${RBDL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RBDL_INCLUDE_DIR}"
    )
    if(RBDL_URDFReader_FOUND AND NOT TARGET RBDL::URDFReader)
        add_library(RBDL::URDFReader UNKNOWN IMPORTED)
        set_target_properties(RBDL::URDFReader PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${RBDL_URDFReader_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${RBDL_URDFReader_INCLUDE_DIR}"
        )
    endif()
endif()

