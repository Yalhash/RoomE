#include "ArduinoSerial.h"
#include <iostream>
#include <unistd.h>
#include <string>


int main () {
    ArduinoSerial serial;
    if (serial.usb_open()) {
	std::cout << "Good open" << std::endl;
    } else {
	std::cout << "BAD open" << std::endl;
    }

    while (1) {
	    std::cout << "give input: ";
	    std::string myStr;
	    std::cin >> myStr;
        serial.write_string(myStr);
        auto str = serial.read_string();
        std::cout << "Arduino sent back " << str << std::endl;
        sleep(2);
    }
    serial.usb_close();

    return 0;
}

