cmake_minimum_required(VERSION 2.6)
project(WAM_UDP)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

option(SERVER "server" ON)
## boost
find_package(Boost 1.46 REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

## wam
include_directories("include/")

if(SERVER)
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

