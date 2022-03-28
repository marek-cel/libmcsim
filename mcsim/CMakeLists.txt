set( TARGET_NAME mcSim )

################################################################################

set( HEADERS
    defs.h
)

set( SOURCES
    mcSim.cpp
)

################################################################################

add_library( ${TARGET_NAME} SHARED ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcSim
)

################################################################################

add_subdirectory( aero )
add_subdirectory( ctrl )
add_subdirectory( envr )
add_subdirectory( gear )
add_subdirectory( mass )
add_subdirectory( prop )

################################################################################

target_link_libraries( ${PROJECT_NAME}
    aero
    ctrl
    envr
    gear
    mass
    prop
)
