#file( GLOB UTILITIES *.cpp )

set(UTILITIES	accButtonTest.cpp
		buzzertest.cpp
		canpack.cpp
		pandacan.cpp
		pandacord.cpp
		pandagps.cpp
		pandaSetSystemTime.cpp
		pandatest.cpp
		pandaversion.cpp
		testGpsLibusb.cpp)

if(DEFINED WIRING_PI_FOUND)
	set(UTILITIES ${UTILITIES} rawAccButtonTest.cpp)
endif()

foreach( utilityfile ${UTILITIES} )
	# Each utility will be the same name (without extension) as the cpp file.
	get_filename_component(utilityname ${utilityfile} NAME_WE )

	#message( "executable named ${utilityname} from file ${utilityfile}" )
	
	add_executable( ${utilityname} ${utilityfile} )

	target_link_libraries ( ${utilityname} ${DEPENDENCIES})

	# Install
	set_target_properties( ${utilityname} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
	install( TARGETS ${utilityname}
	           		DESTINATION bin
	           		)
endforeach( utilityfile ${UTILITIES} )
