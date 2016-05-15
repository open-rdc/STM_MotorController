// Chiba Institute of Technology

#ifndef RS485_H
#define RS485_H

#include "mbed.h"

class RS485 : public Serial
{
public:
	RS485(PinName tx, PinName rx, PinName selectOut);

	int putc(int c);

	int printf(const char* format, ...);

	virtual ssize_t write(const void* buffer, size_t length);
	virtual ssize_t read(void* buffer, size_t length);

private:
//	Serial serial;
	DigitalOut select_out_;
	void txFinishCallback(void);
};

#endif
