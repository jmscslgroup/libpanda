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
		
	install( FILES ${GENERATED_HEADERS}
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

#execute_process(COMMAND git rev-parse --short=8 HEAD
execute_process(COMMAND git rev-parse HEAD
                OUTPUT_VARIABLE LIBPANDA_GIT_REV
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE )
message("Libpanda git version: ${LIBPANDA_GIT_REV}")

execute_process(COMMAND git -C ${CMAKE_SOURCE_DIR}/panda rev-parse --short=8 HEAD
                OUTPUT_VARIABLE PANDA_GIT_REV
                ERROR_QUIET 
                OUTPUT_STRIP_TRAILING_WHITESPACE )
message("Panda git version: ${PANDA_GIT_REV}")

    include_directories( ${CMAKE_CURRENT_BINARY_DIR}
                         ${CMAKE_CURRENT_SOURCE_DIR} )
    configure_file(include/panda_version_i.h.in "${CMAKE_CURRENT_BINARY_DIR}/panda_version_i.h" @ONLY)
    set(GENERATED_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/panda_version_i.h)



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
set(c_sources     ${${LIBNAME}_SOURCES} ${${LIBNAME}_HEADERS} ${GENERATED_HEADERS})
set(c_headers     ${${LIBNAME}_HEADERS} ${GENERATED_HEADERS}/)

# Dependencies:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
find_package(libusb-1.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(Curses)
find_package(Jsoncpp)

if(NOT JSONCPP_FOUND)
	message(" - Warning! JSONCPP not found!")
else()

	include_directories( ${JSONCPP_INCLUDE_DIRS} )
	set( DEPENDENCIES ${DEPENDENCIES} ${JSONCPP_LIBRARIES} )
endif()

include_directories( "include"
                     "NMEAParser/Software/NMEAParserLib/include"
                     ${LIBUSB_1_INCLUDE_DIRS}
                     ${CMAKE_THREAD_LIBS_INIT}  )

set(LIBS  ${LIBS}
          ${LIBUSB_1_LIBRARIES}
          ${CMAKE_THREAD_LIBS_INIT} )

set( DEPENDENCIES ${DEPENDENCIES} ${LIBS} )

set(LIBS ${LIBS} ${LIBUSB_1_LIBRARIES}
                                mogithread
                                NMEAParserLib)

# Check for wiringPi:
if(EXISTS /usr/local/include/wiringPiSPI.h)
	message("adding definition WIRING_PI_FOUND")
	add_definitions(-DWIRING_PI_FOUND)
	set(WIRING_PI_FOUND YES)
        set(LIBS ${LIBS} wiringPi)
endif()



# Libraries:
add_library(mogithread SHARED src/mogi/Thread.cpp include/mogi/thread.h )
add_library(mogithread-static STATIC src/mogi/Thread.cpp include/mogi/thread.h )
target_link_libraries(mogithread ${CMAKE_THREAD_LIBS_INIT} )

add_library(${LIBNAME}  SHARED   ${c_sources} )
add_library(${LIBNAME}-static STATIC ${c_sources})
set_target_properties(${LIBNAME}-static PROPERTIES OUTPUT_NAME "${LIBNAME}")
set_target_properties(${LIBNAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${LIBNAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_link_libraries(${LIBNAME}    ${LIBUSB_1_LIBRARIES} ${LIBS} )

set( DEPENDENCIES ${DEPENDENCIES} ${LIBNAME} ncurses ${LIBS} )

new_install( ${LIBNAME} )
new_install( mogithread )

# Test executables:
if(${CURSES_FOUND})
     add_library(cursesgfx example/curses/cursesGfx.cpp
                           example/curses/curses-gfx.h
                           example/curses/cursesClock.cpp
                           example/curses/curses-clock.h )
     target_link_libraries(cursesgfx ncurses )
	 include_directories(example/curses)

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
                                        NMEAParserLib
                                        ncurses 
                                        cursesgfx )
endif()

add_executable(nissanTest	example/nissanTest.cpp
				example/device.h
				example/deviceTypes.h
				example/joystick.cpp
				example/joystickState.cpp
				example/joystickState.h
				example/device.cpp
				example/mouse.cpp )
target_link_libraries(nissanTest	${LIBS}
									${LIBNAME}
									mogithread
									NMEAParserLib )
									
add_executable(simpleSend	example/simpleSend.cpp
				example/device.h
				example/deviceTypes.h
				example/joystick.cpp
				example/joystickState.cpp
				example/joystickState.h
				example/device.cpp
				example/mouse.cpp )


target_link_libraries(simpleSend	${LIBS}
									${LIBNAME}
									mogithread
									NMEAParserLib )

add_executable(pandaVin		example/pandaVin.cpp )
target_link_libraries(pandaVin ${LIBS}
									${LIBNAME}
									mogithread
									NMEAParserLib)

add_executable(pandaObd		example/pandaObd.cpp )
target_link_libraries(pandaObd ${LIBS}
									${LIBNAME}
									mogithread
									NMEAParserLib)

add_executable(buttonTest	example/buttonTest.cpp )
target_link_libraries(buttonTest ${LIBS}
									${LIBNAME}
									mogithread
									NMEAParserLib)

add_subdirectory(utilities)






