/*
 * A class that uses C constructs to set up communications with the arduino through serial ports
 */

#ifndef ARDUINO_SERIAL_H
#define ARDUINO_SERIAL_H

#include <string>
#include <termios.h>

class ArduinoSerial {
public:
    ArduinoSerial() : usb_desc(-1) {}

    bool usb_open();
    bool _write_char(char c);
    char _read_char();
    bool write_string(const std::string& str);
    std::string read_string();
    bool usb_close();
private:
    const std::string serial_base = "/dev/ttyUSB";
    std::string file_name;
    int usb_desc;
    static const auto SERIAL_BAUDRATE = B115200;
};

#endif
