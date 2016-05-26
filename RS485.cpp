#include "RS485.h"
#include "mbed.h"
#include <stdarg.h>

RS485::RS485(PinName tx, PinName rx, PinName selectOut) :
    serial_(tx, rx), select_out_(selectOut), p_read(0), p_stock(0)
{
	select_out_ = 0;	// input
  serial_.format(8, Serial::None, 1);
	serial_.attach(this, &RS485::txFinishCallback, serial_.TxIrq);
	serial_.attach(this, &RS485::rxFinishCallback, serial_.RxIrq);
}

void RS485::baud(int baudrate){
  serial_.baud(baudrate);
  wait(0.001);
}

int RS485::readable()
{
  return serial_.readable();
}

int RS485::putc(int c)
{
	select_out_ = 1;
	return serial_.putc(c);
}

int RS485::getc()
{
	return serial_.getc();
}

int RS485::printf(const char* format, ...)
{
	select_out_ = 1;
	
	va_list arg;
	va_start(arg, format);
	int res = serial_.vprintf(format, arg);
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
		serial_.putc(*buf++);
	}
	
	return length;
}

ssize_t RS485::read(void* buffer, size_t length)
{
	unsigned char *buf = (unsigned char *)buffer;
  int len = p_stock - p_read;
  if (len < 0) len += MAX_RECV_BUFFER;
  if (len > length) len = length;
	for(int i = 0; i < len; i ++){
    *buf++ = rx_buf[p_read++];
    if (p_read >= MAX_RECV_BUFFER) p_read = 0;
	}
	return len;
}

void RS485::rxFinishCallback(void)
{
  rx_buf[p_stock ++] = serial_.getc();
  if (p_stock == MAX_RECV_BUFFER) p_stock = 0;
}
