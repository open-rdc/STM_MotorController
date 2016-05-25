#include <stdarg.h>
#include "mbed.h"
#include "RS485.h"

RS485::RS485(PinName tx, PinName rx, PinName selectOut) :
		Serial(tx, rx), select_out_(selectOut)
{
	select_out_ = 0;	// input
	attach(this, &RS485::txFinishCallback, Serial::TxIrq);
}

int RS485::putc(int c)
{
	select_out_ = 1;
	int res = Serial::putc(c);
	return res;
}

int RS485::printf(const char* format, ...)
{
	select_out_ = 1;
	
	va_list arg;
	va_start(arg, format);
	fflush(_file);
	int res = vfprintf(_file, format, arg);
	va_end(arg);	
	return res;
}

void RS485::txFinishCallback(void)
{
	select_out_ = 0;
}

ssize_t RS485::write(const void* buffer, size_t length)
{
	char *buf = (char *)buffer;
	for(int i = 0; i < length; i ++){
		this->putc(*buf++);
	}
	
	return length;
}

ssize_t RS485::read(void* buffer, size_t length)
{
	char *buf = (char *)buffer;
	while(select_out_) wait(0.001);	
	for(int i = 0; i < length; i ++){
		*buf++ = getc();
	}
	return length;
}
