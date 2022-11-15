set( TARGET_NAME prop )

################################################################################

set( HEADERS
    PistonEngine.h
    Propeller.h
    PropellerGovernor.h
)

set( SOURCES
    PistonEngine.cpp
    Propeller.cpp
    PropellerGovernor.cpp
)

################################################################################

add_library( ${TARGET_NAME} OBJECT ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/prop
)

