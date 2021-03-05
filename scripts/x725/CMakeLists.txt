cmake_minimum_required(VERSION 3.1)

project(x725)

option( POWER_CONTROL "Enables power management (shutdown/reboot)" ON)

if( POWER_CONTROL )
	add_compile_definitions( POWERCTL )
endif( POWER_CONTROL )

add_library( rpigpio src/rpigpio.h
                     src/rpigpio.cpp )

add_executable( x725available src/x725available.cpp )

add_executable( x725power src/x725power.cpp )

add_executable( x725button src/x725button.cpp )

target_link_libraries( x725button rpigpio )
target_link_libraries( x725power rpigpio )

# Install
set_target_properties( x725power PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install( TARGETS x725power
	         DESTINATION bin )
set_target_properties( x725button PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install( TARGETS x725button
	         DESTINATION bin )
set_target_properties( x725available PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install( TARGETS x725available
	         DESTINATION bin )
