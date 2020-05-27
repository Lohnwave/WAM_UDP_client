cmake_minimum_required(VERSION 2.6)
project(WAM_UDP)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

option(CLIENT "client" OFF)
option(WAM_CLIENT "wam_client" ON)
option(EX06 "ex06" ON)
## boost
find_package(Boost 1.46 REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

## wam
include_directories("include/")

if(CLIENT)
	## libbarrett
	find_package(Barrett REQUIRED)
	include_directories(${BARRETT_INCLUDE_DIRS})
	link_directories(${BARRETT_LIBRARY_DIRS})
	add_definitions(${BARRETT_DEFINITIONS})

	##add_executable(wam_server src/wam_server.cpp)
	add_executable(client src/udp_client.cpp)
	##target_link_libraries(wam_server boost_system boost_filesystem ${BARRETT_LIBRARIES})
	target_link_libraries(client boost_system boost_filesystem ${BARRETT_LIBRARIES})
endif()

if(WAM_CLIENT)
	## libbarrett
	find_package(Barrett REQUIRED)
	include_directories(${BARRETT_INCLUDE_DIRS})
	link_directories(${BARRETT_LIBRARY_DIRS})
	add_definitions(${BARRETT_DEFINITIONS})

	##add_executable(wam_server src/wam_server.cpp)
	add_executable(wam_client src/WAM_client.cpp)
	##target_link_libraries(wam_server boost_system boost_filesystem ${BARRETT_LIBRARIES})
	target_link_libraries(wam_client boost_system boost_filesystem ${BARRETT_LIBRARIES})
endif()

if(EX06)
	## libbarrett
	find_package(Barrett REQUIRED)
	include_directories(${BARRETT_INCLUDE_DIRS})
	link_directories(${BARRETT_LIBRARY_DIRS})
	add_definitions(${BARRETT_DEFINITIONS})

	##add_executable(wam_server src/wam_server.cpp)
	add_executable(ex06 src/ex06_realtime_move.cpp)
	##target_link_libraries(wam_server boost_system boost_filesystem ${BARRETT_LIBRARIES})
	target_link_libraries(ex06 boost_system boost_filesystem ${BARRETT_LIBRARIES})
endif()