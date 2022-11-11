set( TARGET_NAME mcsim-ctrl )

################################################################################

set( HEADERS
    HingeMoment.h
)

set( SOURCES
    HingeMoment.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/ctrl
)

