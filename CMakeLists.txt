cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)

project(libpanda)
add_subdirectory( NMEAParser/Software )


# Dependencies:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
find_package(libusb-1.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(Curses)

include_directories( "include"
                     "NMEAParser/Software/NMEAParserLib"
                     ${LIBUSB_1_INCLUDE_DIRS}
                     ${CMAKE_THREAD_LIBS_INIT} )

set(LIBS  ${LIBS}
          ${LIBUSB_1_LIBRARIES}
          ${CMAKE_THREAD_LIBS_INIT} )


# Libraries:
add_library(mogithread  src/mogi/Thread.cpp
                        include/mogi/thread.h )
target_link_libraries(mogithread ${CMAKE_THREAD_LIBS_INIT} )

add_library(pandac     src/panda/usb.cpp
                       src/panda/usbListener.cpp
                       src/panda/can.cpp
                       src/panda/gps.cpp
                       src/panda/panda.cpp
                       include/panda/usb.h
                       include/panda/can.h
                       include/panda/gps.h
                       include/panda/candata.h
                       include/panda/gpsdata.h
                       include/panda/pandadefinitions.h
                       include/panda.h )
target_link_libraries(pandac    ${LIBUSB_1_LIBRARIES}
                                mogithread
                                NMEAParserLib )

add_library(strym     src/strym.cpp
                      include/strym.h )
target_link_libraries(strym    ${LIBUSB_1_LIBRARIES}
                               mogithread
                               pandac )


# Test executables:
add_executable(listdevs      example/listdevs.c)
add_executable(panda         example/pandaMain.cpp)
add_executable(pandaSetTime  example/pandaSetSystemTime.cpp)
add_executable(simplegps     example/simpleGPS.cpp)
add_executable(simplecan     example/simpleCAN.cpp)


target_link_libraries(listdevs  ${LIBS})
target_link_libraries(panda     ${LIBS}
                                pandac
                                mogithread
                                strym
                                NMEAParserLib )
target_link_libraries(pandaSetTime    ${LIBS}
                                      pandac
                                      mogithread
                                      strym
                                      NMEAParserLib )
target_link_libraries(simplegps    ${LIBS}
                                   pandac
                                   mogithread
                                   strym
                                   NMEAParserLib )
target_link_libraries(simplecan    ${LIBS}
                                   pandac
                                   mogithread
                                   strym
                                   NMEAParserLib )

if(${CURSES_FOUND})
     add_executable(pandaCurses  example/pandaCurses.cpp
                                 example/cursesHandler.cpp
                                 example/cursesHandler.h )

     target_link_libraries(pandaCurses  ${LIBS}
                                        pandac
                                        mogithread
                                        strym
                                        NMEAParserLib
                                        ncurses )
endif()

