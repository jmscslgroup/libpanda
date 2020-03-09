file( GLOB UTILITIES *.cpp )
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
