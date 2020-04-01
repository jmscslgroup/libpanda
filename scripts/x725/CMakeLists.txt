cmake_minimum_required(VERSION 3.1)



project(x725)



add_executable( x725power src/x725power.cpp )

add_executable( x725button src/rpigpio.h
                           src/rpigpio.cpp
                           src/x725button.cpp )

# Install
set_target_properties( x725power PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install( TARGETS x725power
	         DESTINATION bin )
set_target_properties( x725button PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install( TARGETS x725button
	         DESTINATION bin )