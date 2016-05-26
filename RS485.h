// Chiba Institute of Technology

#ifndef RS485_H
#define RS485_H

/** Class to communicate with RS485
 *
 * Exmaple
 * @code
 *
 * #include "mbed.h" 
 * #include "RS485.h" 
 *
 * RS485 rs485(RS485_TX, RS485_RX, RS485_SELECT); 
 * 
 * int main() {
 * 	while(1) {
 *		  rs485.printf("Hello World\r\n");
 *		  wait(0.5);
 *	  }
 * }
 */

#include "mbed.h"

class RS485
{
public:
	RS485(PinName tx, PinName rx, PinName selectOut);

	void baud(int baudrate);

  int readable();

  int putc(int c);

  int getc();

  int printf(const char* format, ...);

	virtual ssize_t write(const void* buffer, size_t length);
	virtual ssize_t read(void* buffer, size_t length);

private:
  Serial serial_;
	DigitalOut select_out_;
	void txFinishCallback(void);
};

#endif

