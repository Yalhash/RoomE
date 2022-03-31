#include "ArduinoSerial.h"
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>

bool ArduinoSerial::usb_open() {
	int bits;
	struct termios term;
	struct serial_struct kernel_serial_settings;
    if ((usb_desc = open(file_name.c_str(), 
        O_RDWR | O_NONBLOCK | O_NOCTTY )) == -1) {
		fprintf(stderr, "opening USB connection '%s' failed: %s\n", file_name.c_str(), strerror(errno));
        usb_desc = -1;
		return false;
    }


	/* 
	 * Reset the Arduino's line. This is key to getting the write to work.
	 * Without it, the first few writes don't work.
	 * Clear DTR, wait one second, flush input, then set DTR.
	 * Without this, the first write fails.
 	*/
	if (ioctl(usb_desc, TIOCMGET, &bits) < 0) {
		close(usb_desc);
		perror("ioctl(TCIOMGET)");
        usb_desc = -1;
		return false;
	}
	bits &= ~(TIOCM_DTR | TIOCM_RTS);
	if (ioctl(usb_desc, TIOCMSET, &bits) < 0) {
		close(usb_desc);
		perror("ioctl(TIOCMSET)");
        usb_desc = -1;
		return false;
	}
	sleep(2); // NOTE: when this was 1 second sleep it did not work. 
	tcflush(usb_desc, TCIFLUSH);
	bits &= TIOCM_DTR;
	if (ioctl(usb_desc, TIOCMSET, &bits) < 0) {
		close(usb_desc);
		perror("ioctl(TIOCMSET)");
        usb_desc = -1;
		return false;
	}

	memset(&term, 0, sizeof(term));
	term.c_iflag = IGNBRK | IGNPAR;
	term.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;
	cfsetospeed(&term, SERIAL_BAUDRATE); 
	cfsetispeed(&term, SERIAL_BAUDRATE);
	if (tcsetattr(usb_desc, TCSANOW, &term) < 0) {
		perror("tcsetattr()");
        usb_desc = -1;
		return false;
	}
	if (ioctl(usb_desc, TIOCGSERIAL, &kernel_serial_settings) == 0) {
		kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
		ioctl(usb_desc, TIOCSSERIAL, &kernel_serial_settings);
	}
	tcflush(usb_desc, TCIFLUSH);

    return true;
}

bool ArduinoSerial::usb_close() {
    int err = 0;
    if (usb_desc != -1) {
        err = close(usb_desc);
    }

    usb_desc = -1;
    return err == 0; 
}

bool ArduinoSerial::write_string(const std::string& str) {
	ssize_t n;
	const char* c_buf = str.c_str();
	if ((n = write(usb_desc, c_buf, str.size())) == -1) {
		fprintf(stderr, "write() failed: %s\n", strerror(errno));
		return false;
	}
    return true;
}

std::string ArduinoSerial::read_string() {
    char buf[100]; // NOTE: Arbitrary
	int n;

	while (1) {
		if ((n = read(usb_desc, buf, 50)) == -1) {
			if (errno != EAGAIN) {
				fprintf(stderr, "read() failed: (%d) %s\n", errno, strerror(errno));
				return "";
			}
			// errno == EAGAIN, loop around and read again
		} else if (n != 0) {
			return std::string(buf); // stop reading
		}
		// read 0 bytes, loop around an read again
	}
}
