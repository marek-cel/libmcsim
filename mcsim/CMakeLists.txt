set( TARGET_NAME mcsim )

################################################################################

set( HEADERS
    defs.h
)

set( SOURCES
    mcsim.cpp
)

################################################################################

add_library( ${TARGET_NAME} SHARED ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    VERSION ${MCSIM_VERSION_STRING}
    SOVERSION ${MCSIM_VERSION_MAJOR}
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim
)

################################################################################

add_subdirectory( aero )
add_subdirectory( env )
add_subdirectory( ext )
add_subdirectory( gear )
add_subdirectory( prop )
add_subdirectory( rotor )
add_subdirectory( utils )

################################################################################

target_link_libraries( ${TARGET_NAME}
    mcsim-aero
    mcsim-env
    mcsim-ext
    mcsim-gear
    mcsim-prop
    mcsim-rotor
    mcsim-utils
)
