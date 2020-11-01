
find_path(qpOASES_INCLUDE_DIR qpOASES.hpp
    HINTS
        $ENV{qpOASES_INCLUDE_PATH}
        $ENV{qpOASES_PATH}/include
        /usr/local/include
        /usr/include
)

find_library(qpOASES_LIBRARY NAMES qpOASES
    PATHS
        $ENV{qpOASES_LIBRARY_PATH}
        $ENV{qpOASES_PATH}/lib
        /usr/local/lib
        /usr/lib
)

if(NOT qpOASES_LIBRARY)
    message(SEND_ERROR "Not found qpOASES library")
endif()

if(qpOASES_LIBRARY AND qpOASES_INCLUDE_DIR)
    set(qpOASES_FOUND TRUE)
endif()

mark_as_advanced(
    qpOASES_INCLUDE_DIR
    qpOASES_LIBRARY
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(qpOASES
    REQUIRED_VARS
        qpOASES_INCLUDE_DIR
        qpOASES_LIBRARY
)

if(qpOASES_FOUND AND NOT TARGET qpOASES::qpOASES)
    add_library(qpOASES::qpOASES UNKNOWN IMPORTED)
    set_target_properties(qpOASES::qpOASES PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${qpOASES_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${qpOASES_INCLUDE_DIR}"
    )
endif()

