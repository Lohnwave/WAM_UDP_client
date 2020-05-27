#include <boost/thread.hpp> // for thread udp
#include <boost/thread/mutex.hpp> // for thread udp
#include <string>
#include <iostream>

std::string str("123");
void getCin() {
    while(true)
        std::cin >> str;
}
void print() {
    std::cout << str << std::endl;
}
int main() {
    boost::thread* get = NULL;
    get = new boost::thread(getCin);
    get->detach();
    while(true) 
        print();

    return 0;
}