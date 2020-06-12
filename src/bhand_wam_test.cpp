/******************************************************************************\
* Copyright (C) 2020  江西省智能信息系统重点实验室, All rights reserved.          *
* Version: WAM 3.0                                                                 *
* Last Revised: 2020-06-01                                                     *
* Editor: Luozu	                                                               *
* v 3.0: WAM和Bhand同时控制。获得服务器发送的WAM 7个关节角以及Bhand 2个手指间隔     *
*                           前者控制WAM位姿，后者控制Bhand抓握                    *
\******************************************************************************/
#include <iostream>
#include <string>
#include <vector>
#include <math.h> // for fabs

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
const size_t JOINT_NUM = 9;// WAM 7 joints and bhand 4 motors
std::vector<double> rev_data(JOINT_NUM); // udp get data
//std::vector<double> bh_j(4); // bhand F1 F2 F3 spread motors


enum { max_length = 1024 };

void initJp() {
	rev_data[0] = 0.019;
	rev_data[1] = 0.719;
	rev_data[2] = -0.055;
	rev_data[3] = 2.210;
	rev_data[4] = 0.113;
	rev_data[5] = 0.208;
	rev_data[6] = -0.110;
	rev_data[7] = 130;
	rev_data[8] = 170;
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
		jp[0] = rev_data[0];
		jp[1] = rev_data[1];
		jp[2] = rev_data[2];
		jp[3] = rev_data[3];
		jp[4] = rev_data[4];
		jp[5] = rev_data[5];
		jp[6] = rev_data[6];
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
        sock.receive_from(
          boost::asio::buffer(data, max_length), sender_endpoint);
		size_t msglen = strlen(data);

		size_t i = 0, jcount = 0;
		std::string temp;
		jpw_mutex.lock();
		while(i<msglen&&jcount<JOINT_NUM) {
			if (data[i]==',') {
			rev_data[jcount] = atof(temp.c_str());
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

void bhandMove(Hand& hand) {
	Hand::jp_type fj_val;
	bool going = true;
	while(going) {
		jpw_mutex.lock();
		//fj_val[0] = fabs(M_PI_2*(130-rev_data[7])/130);
		//fj_val[1] = fabs(M_PI_2*(170-rev_data[8])/170);
		fj_val[0] = fabs(2*(130-rev_data[7])/130); // 130 mm this distance between F0 and F1
		//fj_val[1] = fabs(2*(170-rev_data[8])/170);
		fj_val[1] = fj_val[0];
		fj_val[2] = fj_val[0]>fj_val[1]?fj_val[0]:fj_val[1]; // 
		fj_val[3] = 0;
		jpw_mutex.unlock();
		hand.trapezoidalMove(fj_val);
		//hand.setPositionCommand(fj_val);
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
        if (!pm.foundHand()) {
            std::cerr << "ERROR: No hand found...";
            return 1;
        }
		const double TRANSITION_DURATION = 0.5;  // seconds
		const double JP_AMPLITUDE = 0.4;  // radians
		const double FREQUENCY = 2.0;  // rad/s 0527

		//Rate Limiter
		jp_type rt_jp_cmd;
		systems::RateLimiter<jp_type> jp_rl;
		//Sets the joints to move at 1 m/s
		const double rLimit[] = {0.1, 0.08, 0.1, 0.1, 0.5, 0.5, 0.5};// 20200527 

		for(size_t i = 0; i < DOF; ++i)
			rt_jp_cmd[i] = rLimit[i];

		// Set start position, depending on robot type and configuration.
		jp_type startPos(0.0);
		startPos[0] = 0.019;
		startPos[1] = 0.719;
		startPos[2] = -0.055;
		startPos[3] = 2.210;
		startPos[4] = 0.113;
		startPos[5] = 0.208;
		startPos[6] = -0.110;
		initJp(); // init th_jp
		systems::Ramp time(pm.getExecutionManager(), 1.0);

		printf("Press [Enter] toinitialize joint position control.");
		waitForEnter();
		
		wam.moveTo(startPos); 
		Hand& hand = *pm.getHand();
		hand.initialize();
		//hand.close();
		//hand.open();
		printf("Press [Enter] to begin WAM move");
		waitForEnter();
		
		boost::thread* wamThread = NULL;
		boost::thread* bhThread = NULL;
		wamThread = new boost::thread(sockGetjp, atoi(argv[1]));
		wamThread->detach();
		
		bhThread = new boost::thread(bhandMove, boost::ref(hand));
		bhThread->detach();
		//wam.moveTo(startPos);
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
		delete wamThread;
		delete bhThread;
		hand.close("");
		time.smoothStop(TRANSITION_DURATION);
		wam.idle();
		
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
