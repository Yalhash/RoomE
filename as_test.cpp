#include "ArduinoSerial.h"
#include <iostream>
#include <unistd.h>
#include <string>


int main () {
    ArduinoSerial serial;
    serial.usb_open();

    while (1) {
        std::cout << "Telling the Arduino to start blinking..." << std::endl;

		if (!serial._write_char('1')) {
            std::cout << "Write failed!" << std::endl;
            return -1;
		}

		// read to get the acknowledgement from the Arduino

        char c = serial._read_char();
        if (c == '\0') {
            std::cout << "Read failed!" << std::endl;

        }

        std::cout << "Arduino sent back " << c << std::endl;

		sleep(2);

        std::cout << "Telling the Arduino to stop blinking..." << std::endl;

		if (!serial._write_char('0')) {
            std::cout << "Write2 failed!" << std::endl;
            return -1;
		}

		// read to get the acknowledgement from the Arduino
        c = serial._read_char();

		if (c == '\0') {
            std::cout  << "second read failed!" << std::endl;
            return -1;
		}
        std::cout << "Arduino sent back " << c << std::endl;

		sleep(2);

    }
    serial.usb_close();

    return 0;
}

