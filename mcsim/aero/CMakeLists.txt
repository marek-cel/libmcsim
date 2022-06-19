set( TARGET_NAME mcsim-aero )

################################################################################

set( HEADERS
    Fuselage.h
    StabilizerHor.h
    StabilizerVer.h
    TailOff.h
)

set( SOURCES
    Fuselage.cpp
    StabilizerHor.cpp
    StabilizerVer.cpp
    TailOff.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/aero
)

