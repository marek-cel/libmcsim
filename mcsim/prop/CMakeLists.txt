set( TARGET_NAME mcsim_prop )

################################################################################

set( HEADERS
    Governor.h
    PistonEngine.h
    Propeller.h
)

set( SOURCES
    Governor.cpp
    PistonEngine.cpp
    Propeller.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/prop
)

