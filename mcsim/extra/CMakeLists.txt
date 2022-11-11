set( TARGET_NAME mcsim-extra )

################################################################################

set( HEADERS
    WinchLauncher.h
    WingRunner.h
)

set( SOURCES
    WinchLauncher.cpp
    WingRunner.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/extra
)

