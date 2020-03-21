cmake_minimum_required(VERSION 3.1)

function( new_install LIBNAME )
	# Install
	if(BUILD_SHARED_LIBS)
		install( TARGETS ${LIBNAME}
			LIBRARY DESTINATION lib #lib${LIB_SUFFIX}
			COMPONENT sharedlibs
			)
	endif(BUILD_SHARED_LIBS)

	if(BUILD_STATIC_LIBS)
		install( TARGETS ${LIBNAME}-static
			ARCHIVE DESTINATION lib #lib${LIB_SUFFIX}
			COMPONENT staticlibs
			)
	endif(BUILD_STATIC_LIBS)

	install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ #${c_headers}
		DESTINATION include
		COMPONENT headers
		)

	message("-- Install Directory ${LIBNAME}: lib${LIB_SUFFIX}")
	message("-- Install Directory ${LIBNAME}-static: lib${LIB_SUFFIX}")
	message("-- Install Directory ${c_headers}: include/${LIB_SUFFIX}")
endfunction()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_COLOR_MAKEFILE ON)

set(CMAKE_SKIP_BUILD_RPATH false)
set(CMAKE_BUILD_WITH_INSTALL_RPATH false)
#SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)

if(APPLE)
	set(CMAKE_MACOSX_RPATH 1)
endif()

project(libpandac)
set(LIBNAME pandac)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Create suffix to eventually install in lib64
IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(LIB_SUFFIX "")
    SET(PACK_ARCH "")
  ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(LIB_SUFFIX 64)
    SET(PACK_ARCH .x86_64)
endif(CMAKE_SIZEOF_VOID_P EQUAL 4)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")#${LIB_SUFFIX}")

option( BUILD_SHARED_LIBS "Build the shared library" ON)
option( BUILD_STATIC_LIBS "Build the static library" ON)


add_subdirectory( NMEAParser/Software )


file(GLOB_RECURSE ${LIBNAME}_SOURCES "src/*.cpp")
file(GLOB_RECURSE ${LIBNAME}_HEADERS "include/*.h")
set(c_sources     ${${LIBNAME}_SOURCES} ${${LIBNAME}_HEADERS})
set(c_headers     ${${LIBNAME}_HEADERS})

# Dependencies:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
find_package(libusb-1.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(Curses)

include_directories( "include"
                     "NMEAParser/Software/NMEAParserLib/include"
                     ${LIBUSB_1_INCLUDE_DIRS}
                     ${CMAKE_THREAD_LIBS_INIT} )

set(LIBS  ${LIBS}
          ${LIBUSB_1_LIBRARIES}
          ${CMAKE_THREAD_LIBS_INIT} )

set( DEPENDENCIES ${DEPENDENCIES} ${LIBS} )


# Libraries:
add_library(mogithread SHARED src/mogi/Thread.cpp include/mogi/thread.h )
add_library(mogithread-static STATIC src/mogi/Thread.cpp include/mogi/thread.h )
target_link_libraries(mogithread ${CMAKE_THREAD_LIBS_INIT} )

add_library(${LIBNAME}  SHARED   ${c_sources} )
add_library(${LIBNAME}-static STATIC ${c_sources})
set_target_properties(${LIBNAME}-static PROPERTIES OUTPUT_NAME "${LIBNAME}")
set_target_properties(${LIBNAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${LIBNAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_link_libraries(${LIBNAME}    ${LIBUSB_1_LIBRARIES}
                                mogithread
                                NMEAParserLib )

set( DEPENDENCIES ${DEPENDENCIES} ${LIBNAME} NMEAParserLib ncurses mogithread )

add_library(strym     src/strym.cpp
                      include/strym.h )
target_link_libraries(strym    ${LIBUSB_1_LIBRARIES}
                               mogithread
                               ${LIBNAME} )

new_install( ${LIBNAME} )
new_install( mogithread )

# Test executables:
if(${CURSES_FOUND})
     add_executable(pandaCurses  example/pandaCurses.cpp
                                 example/cursesHandler.cpp
                                 example/cursesHandler.h
                                 example/canFrameStats.cpp
                                 example/canFrameStats.h
                                 example/usbStats.cpp
                                 example/usbStats.h )

     target_link_libraries(pandaCurses  ${LIBS}
                                        ${LIBNAME}
                                        mogithread
                                        strym
                                        NMEAParserLib
                                        ncurses )
endif()

add_subdirectory(utilities)





