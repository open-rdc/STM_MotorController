#include "RS485.h"
#include "mbed.h"
#include <stdarg.h>

RS485::RS485(PinName tx, PinName rx, PinName selectOut) :
    serial_(tx, rx), select_out_(selectOut), guard_time_us_(200), p_read(0), p_stock(0), isUnderPrintString(false)
{
  serial_.baud(115200);
  select_out_ = 0;  // input
  serial_.format(8, Serial::None, 1);
  serial_.attach(this, &RS485::txFinishCallback, serial_.TxIrq);
  serial_.attach(this, &RS485::rxFinishCallback, serial_.RxIrq);
  recv_timer_.start();
  send_timer_.start();
}

void RS485::baud(int baudrate){
  serial_.attach(NULL, serial_.TxIrq);
  serial_.attach(NULL, serial_.RxIrq);
  serial_.baud(baudrate);
  wait(0.001);
  serial_.attach(this, &RS485::txFinishCallback, serial_.TxIrq);
  serial_.attach(this, &RS485::rxFinishCallback, serial_.RxIrq);
}

void RS485::guardTime(int guard_time_us){
    guard_time_us_ = guard_time_us;
}

int RS485::readable()
{
  return serial_.readable();
}

bool RS485::isEnableSend()
{
  if ((select_out_ == 0) && (recv_timer_.read_us() < guard_time_us_)) return false;
  return true;
}

int RS485::putc(int c)
{
  select_out_ = 1;
  return serial_.putc(c);
}

int RS485::getc()
{
  select_out_ = 0;
  return serial_.getc();
}

int RS485::printf(const char* format, ...)
{
  select_out_ = 1;
  isUnderPrintString = true;
  
  va_list arg;
  va_start(arg, format);
  int res = serial_.printf(format, arg);
  va_end(arg);  
  isUnderPrintString = false;
  return res;
}

void RS485::txFinishCallback(void)
{
  send_timer_.reset();
  send_timer_.start();
  if (!isUnderPrintString) select_out_ = 0;
}

ssize_t RS485::write(const void* buffer, size_t length)
{
  char *buf = (char *)buffer;
  for(int i = 0; i < length; i ++){
    serial_.putc((int)*buf++);
  }
  
  return length;
}

ssize_t RS485::read(void* buffer, size_t length)
{
  if (recv_timer_.read_us() < (guard_time_us_ / 2)) return 0;
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
  if ((select_out_ == 1) || (send_timer_.read_us() < guard_time_us_)){
    serial_.getc();
    p_read = p_stock;
  } else {
    rx_buf[p_stock ++] = serial_.getc();
    if (p_stock == MAX_RECV_BUFFER) p_stock = 0;
    recv_timer_.reset();
    recv_timer_.start();
  }
}
