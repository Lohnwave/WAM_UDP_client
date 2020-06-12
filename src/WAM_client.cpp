/******************************************************************************\
* Copyright (C) 2020  江西省智能信息系统重点实验室, All rights reserved.          *
* Version: WAM 1.0                                                                 *
* Last Revised: 2020-06-01                                                     *
* Editor: Luozu	                                                               *
* v 1.0: WAM控制。获得服务器发送的WAM 7个关节角                                   *
*                采用MoveTostr 固定角速度，因此机械臂移动缓慢                     *
\******************************************************************************/

#include <boost/asio.hpp>
#include <vector>
#include <csignal>

#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using boost::asio::ip::udp;

bool __exit_flag = true;
enum { max_length = 1024 };

void msgrebulit(std::string & line, char *data) {
  size_t msglen = strlen(data);
  size_t jointnum = 7;
  size_t i = 0, jcount = 0;
  while(i<msglen&&jcount<7) {
    if (data[i]==',') {
      line += ' ';
      jcount++;
    } else {
      line += data[i];
    }
    i++;
  }
}

void __exit(int sig) {
  __exit_flag = false;
}



// This function template will accept a math::Matrix with any number of rows,
// any number of columns, and any units. In other words: it will accept any
// barrett::units type.
template<int R, int C, typename Units>
bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str) {
	const char* cur = str.c_str();
	const char* next = cur;

	for (int i = 0; i < dest->size(); ++i) {
		(*dest)[i] = strtod(cur, (char**) &next);
		if (cur == next) {
			return false;
		} else {
			cur = next;
		}
	}

	// Make sure there are no extra numbers in the string.
	double ignore = strtod(cur, (char**) &next);
	(void)ignore;  // Prevent unused variable warnings

	if (cur != next) {
		return false;
	}

	return true;
}

template<size_t DOF, int R, int C, typename Units>
void moveToStr(systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving to " << description << ": " << *dest << std::endl;
		// wam.moveTo(*dest);
	} else {
		printf("ERROR: Please enter exactly %ld numbers separated by "
				"whitespace.\n", dest->size());
	}
}

void printMenu() {
	printf("Commands:\n");
	printf("  j  Enter a joint position destination\n");
	printf("  p  Enter a tool position destination\n");
	printf("  h  Move to the home position\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
}
/*
template<size_t DOF, int R, int C, typename Units>
void server(boost::asio::io_service& io_service, short port, systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest) {
  udp::socket sock(io_service, udp::endpoint(udp::v4(), port));
  std::cout << "waiting connect..." << std::endl;
  signal(SIGINT,__exit);
  while (__exit_flag) {
    char data[max_length];
    udp::endpoint sender_endpoint;
    size_t length = sock.receive_from(
        boost::asio::buffer(data, max_length), sender_endpoint);
    //sock.send_to(boost::asio::buffer(data, length), sender_endpoint);
    //std::cout << data << std::endl;
    //print(msg2double(data));
    moveToStr(wam, &jp, "joint positions", data.substr(1));
    //std::cout << data.substr(1) << std::endl;
  }
}
*/
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	// The macro below makes a number of typedefs that allow convenient access
	// to commonly used barrett::units. For example, the typedefs establish
	// "jp_type" as a synonym for "units::JointPositions<DOF>::type". This macro
	// (along with a few others) is defined in barrett/units.h.
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// These vectors are fixed sized, stack allocated, and zero-initialized.
	jp_type jp;  // jp is a DOFx1 column vector of joint positions
	cp_type cp;  // cp is a 3x1 vector representing a Cartesian position


	wam.gravityCompensate();
	// printMenu();
  std::string initPos;
  initPos += "0 0 0 0 0 0 0";
	std::string line;
	bool going = true;

  try {
    if (argc != 2) {
      std::cerr << "Usage: blocking_udp_echo_server <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    using namespace std; // For atoi.
    cout << "running WAM (Y/N) ?" << endl;
    if(cin.get()=='Y') {
        moveToStr(wam, &jp, "joint positions", initPos);
        //server(io_service, atoi(argv[1]), wam, &jp); 
        udp::socket sock(io_service, udp::endpoint(udp::v4(), atoi(argv[1])));
        std::cout << "waiting connect..." << std::endl;
        signal(SIGINT,__exit);
        while (__exit_flag) {
          char data[max_length];
          udp::endpoint sender_endpoint;
          size_t length = sock.receive_from(
              boost::asio::buffer(data, max_length), sender_endpoint);
          //sock.send_to(boost::asio::buffer(data, length), sender_endpoint);
          //std::cout << data << std::endl;
          //print(msg2double(data));
          msgrebulit(line, data);

          moveToStr(wam, &jp, "joint positions", line);
          //std::cout << data.substr(1) << std::endl;
          line.clear();
        }
      }
      else {
        cout << "exit..." << endl;
      }
      
  }
  catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  std::cout << "Moving to home position: "
			<< wam.getHomePosition() << std::endl;
	wam.moveHome();
  std::cout << "exit..." << std::endl;

	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
