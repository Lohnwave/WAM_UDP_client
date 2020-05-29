cmake_minimum_required(VERSION 2.6)
project(WAM_UDP)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

option(THREADTEST "threadtest" OFF)
option(WAM_CLIENT "wam_client" OFF)
option(WAM_UDP_JP_REALTIME "wam_udp_jp_realtime" ON)
option(BHAND_WAM_TEST "bhand_wam_test" ON)

## boost
find_package(Boost 1.46 REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

## wam
include_directories("include/")

if(THREADTEST)
	## libbarrett
	find_package(Barrett REQUIRED)
	include_directories(${BARRETT_INCLUDE_DIRS})
	link_directories(${BARRETT_LIBRARY_DIRS})
	add_definitions(${BARRETT_DEFINITIONS})

	##add_executable(wam_server src/wam_server.cpp)
	add_executable(threadtest src/threadtest.cpp)
	##target_link_libraries(wam_server boost_system boost_filesystem ${BARRETT_LIBRARIES})
	target_link_libraries(threadtest boost_system boost_filesystem ${BARRETT_LIBRARIES})
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

if(WAM_UDP_JP_REALTIME)
	## libbarrett
	find_package(Barrett REQUIRED)
	include_directories(${BARRETT_INCLUDE_DIRS})
	link_directories(${BARRETT_LIBRARY_DIRS})
	add_definitions(${BARRETT_DEFINITIONS})

	##add_executable(wam_server src/wam_server.cpp)
	add_executable(wam_udp_jp_realtime src/wam_udp_jp_realtime.cpp)
	##target_link_libraries(wam_server boost_system boost_filesystem ${BARRETT_LIBRARIES})
	target_link_libraries(wam_udp_jp_realtime boost_system boost_filesystem ${BARRETT_LIBRARIES})
endif()

if(BHAND_WAM_TEST)
	## libbarrett
	find_package(Barrett REQUIRED)
	include_directories(${BARRETT_INCLUDE_DIRS})
	link_directories(${BARRETT_LIBRARY_DIRS})
	add_definitions(${BARRETT_DEFINITIONS})

	##add_executable(wam_server src/wam_server.cpp)
	add_executable(bhand_wam_test src/bhand_wam_test.cpp)
	##target_link_libraries(wam_server boost_system boost_filesystem ${BARRETT_LIBRARIES})
	target_link_libraries(bhand_wam_test boost_system boost_filesystem ${BARRETT_LIBRARIES})
endif()