/******************************************************************************\
* Copyright (C) 2020  江西省智能信息系统重点实验室, All rights reserved.          *
* Version: 3.0                                                                 *
* Last Revised: 2020-06-01                                                     *
* Editor: Luozu	                                                               *
* v 2.0: WAM控制。获得服务器发送的WAM 7个关节角                                   *
*                采用systems实时系统 控制个关节角速度                             *
\******************************************************************************/
#include <iostream>
#include <string>
#include <vector>

#include <boost/asio.hpp> // for udp
#include <boost/thread.hpp> // for thread udp
#include <boost/thread/mutex.hpp> // for thread udp

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h> // for Enter inter

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using boost::asio::ip::udp;

boost::mutex jpw_mutex;
std::vector<double> th_j(7);
enum { max_length = 1024 };
void initJp() {
    th_j[0] = -0.20;
    th_j[1] = 0.48;
    th_j[2] = 0.30;
    th_j[3] = 2.28;
    th_j[4] = -3.10;
    th_j[5] = 1.16;
    th_j[6] = 2.98;
}

template <size_t DOF>
class JpCircle : public systems::SingleIO<double, typename units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	JpCircle(jp_type startPos, double amplitude, double omega, const std::string& sysName = "JpCircle") :
		systems::SingleIO<double, jp_type>(sysName), jp(startPos), jp_0(jp), amp(amplitude), omega(omega) {
			// Check for robot type
			if (DOF>3) {
				// WAM - Use joints 3 and 4.
				i1 = 2;
				i2 = 3;
			} else if (DOF==3) {
				// Proficio - Use joints 2 and 3.
				i1 = 1;
				i2 = 2;
			} else {
				// This should never happen, since DOF check is done in main.
				std::cout << "Warning: No known robot with DOF < 3." << std::endl;
				i1 = 0;
				i2 = 0;
				amp = 0;
			}

		}
	virtual ~JpCircle() { this->mandatoryCleanUp(); }

protected:
	jp_type jp;
	jp_type jp_0;
	double amp, omega;
	double theta;
	int i1, i2;

	virtual void operate() {
//		theta = omega * this->input.getValue();
		// jpw_mutex.lock();
		jp[0] = th_j[0];
		jp[1] = th_j[1];
		jp[2] = th_j[2];
		jp[3] = th_j[3];
		jp[4] = th_j[4];
		jp[5] = th_j[5];
		jp[6] = th_j[6];
		// jpw_mutex.unlock();
		// jp[i1] = amp * std::sin(theta) + jp_0[i1];
		// jp[i2] = amp * (std::cos(theta) - 1.0) + jp_0[i2];

		this->outputValue->setData(&jp);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JpCircle);
};

void sockGetjp(short port) {
	boost::asio::io_service io_service;
	udp::socket sock(io_service, udp::endpoint(udp::v4(), port));
	bool going = true;
    while (going) {
        char data[max_length];
        udp::endpoint sender_endpoint;
        size_t length = sock.receive_from(
          boost::asio::buffer(data, max_length), sender_endpoint);
		size_t msglen = strlen(data);
		size_t jointnum = 7;
		size_t i = 0, jcount = 0;
		std::string temp;
		jpw_mutex.lock();
		while(i<msglen&&jcount<7) {
			if (data[i]==',') {
			th_j[jcount] = atof(temp.c_str());
			temp.clear();
			jcount++;
			} else {
			temp += data[i];
			}
			i++;
		}
		jpw_mutex.unlock();
    }
}
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();
  	try {
		if (argc != 2) {
			std::cerr << "Usage: blocking_udp_echo_server <port>\n";
      		return 1;
    	}

		const double TRANSITION_DURATION = 0.5;  // seconds
		const double JP_AMPLITUDE = 0.4;  // radians
		const double FREQUENCY = 2.0;  // rad/s 0527

		//Rate Limiter
		jp_type rt_jp_cmd;
		systems::RateLimiter<jp_type> jp_rl;
		//Sets the joints to move at 1 m/s
		const double rLimit[] = {0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5};// 20200527 

		for(size_t i = 0; i < DOF; ++i)
			rt_jp_cmd[i] = rLimit[i];

		// Set start position, depending on robot type and configuration.
		jp_type startPos(0.0);
        startPos[0] = -0.2;
        startPos[1] = 0.48;
        startPos[2] = 0.30;
        startPos[3] = 2.28;
        startPos[4] = -3.10;
        startPos[5] = 1.16;
        startPos[6] = 2.98;
        initJp(); // init th_jp
		
		systems::Ramp time(pm.getExecutionManager(), 1.0);

		printf("Press [Enter] to move the end-point in circles using joint position control.");
		waitForEnter();

		boost::thread* ghcThread = NULL;
		ghcThread = new boost::thread(sockGetjp, atoi(argv[1]));
		ghcThread->detach();

		wam.moveTo(startPos);
		//Indicate the current position and the maximum rate limit to the rate limiter
		jp_rl.setCurVal(wam.getJointPositions());
		jp_rl.setLimit(rt_jp_cmd);

		JpCircle<DOF> jpc(startPos, JP_AMPLITUDE, FREQUENCY);

		systems::connect(time.output, jpc.input);
		//Enforces that the individual joints move less than or equal to the above mentioned rate limit
		systems::connect(jpc.output, jp_rl.input);
		wam.trackReferenceSignal(jp_rl.output);
		time.smoothStart(TRANSITION_DURATION);

		printf("Press [Enter] to stop.");
		waitForEnter();
		time.smoothStop(TRANSITION_DURATION);
		wam.idle();
		delete ghcThread;
	}
	catch (std::exception& e) {
    	std::cerr << "Exception: " << e.what() << "\n";
  	}
	printf("Press [Enter] to return home.");
	waitForEnter();
	wam.moveHome();
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
