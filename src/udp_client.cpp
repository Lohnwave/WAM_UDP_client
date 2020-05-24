//
// blocking_udp_echo_server.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <string>
#include <csignal>


using boost::asio::ip::udp;
bool __exit_flag = true;

enum { max_length = 1024 };

double str2double(std::string &str){
  double res = atof(str.c_str());
}
std::vector<double> msg2double(char *data) {
  std::vector<double> joint;
  size_t msglen = strlen(data);
  size_t jointnum = 7;
  size_t i = 0, jcount = 0;
  std::string temp;
  while(i<msglen&&jcount<7) {
    if (data[i]==',') {
      joint.push_back(str2double(temp));
      temp.clear();
      jcount++;
    } else {
      temp += data[i];
    }
    i++;
  }
  return joint;
}
void print(std::vector<double> joint) {
  size_t len = joint.size();
  for(size_t i=0; i<len; i++) 
    std::cout << joint[i] << ",";
  std::cout << "\n";
}
void __exit(int sig) {
  __exit_flag = false;
}
void server(boost::asio::io_service& io_service, short port) {
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
    print(msg2double(data));
  }
}

int main(int argc, char* argv[])
{
  try {
    if (argc != 2) {
      std::cerr << "Usage: blocking_udp_echo_server <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    using namespace std; // For atoi.
    cout << "running WAM (Y/N) ?" << endl;
    if(cin.get()=='Y')
      server(io_service, atoi(argv[1]));
      else {
        cout << "exit..." << endl;
      }
      
  }
  catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
  std::cout << "exit..." << std::endl;
  return 0;
}
