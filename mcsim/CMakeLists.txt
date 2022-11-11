set( TARGET_NAME mcsim )

################################################################################

set( HEADERS
    defs.h
)

set( SOURCES
    mcsim.cpp
)

################################################################################

add_library( ${TARGET_NAME} ${SOURCES} )

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
add_subdirectory( ctrl )
add_subdirectory( env )
add_subdirectory( extra )
add_subdirectory( gear )
add_subdirectory( mass )
add_subdirectory( prop )
add_subdirectory( rotor )

################################################################################

target_link_libraries( ${TARGET_NAME}
    mcsim-aero
    mcsim-ctrl
    mcsim-env
    mcsim-extra
    mcsim-gear
    mcsim-mass
    mcsim-prop
    mcsim-rotor
)
