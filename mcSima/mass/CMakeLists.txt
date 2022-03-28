set( TARGET_NAME mass )

################################################################################

set( HEADERS
    InertiaMatrix.h
    VariableMass.h
)

set( SOURCES
    InertiaMatrix.cpp
    VariableMass.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/mass
)

