#include "RS485.h"
#include <stdarg.h>

#define SEND_START_SIGNAL 1
#define SEND_END_SIGNAL 2

RS485::RS485(PinName tx, PinName rx, PinName selectOut) :
  _serial(tx, rx), _select_out(selectOut),
  _guard_time_us(200.0f),
  _recv_thread(&RS485::recvThreadStarter, this, osPriorityNormal), _recv_buf(MAX_RECV_BUFFER),
  _send_thread(&RS485::sendThreadStarter, this, osPriorityNormal), _send_buf(MAX_SEND_BUFFER)
{
  _serial.baud(1500000);
  _select_out = 0;  // input
  _serial.format(8, Serial::None, 1);
  _recv_timer.start();
  _send_timer.start();
}

void RS485::baud(int baudrate){
  _serial.baud(baudrate);
  Thread::wait(1);
}

void RS485::guardTime(int guard_time_us){
    _guard_time_us = guard_time_us;
}

int RS485::readable()
{
  return _serial.readable();
}

bool RS485::isEnableSend()
{
  return !_send_buf.full();
}

int RS485::putc(int c)
{
  int res = (_send_buf.write(c)) ? 1 : 0;
  _send_thread.signal_set(SEND_START_SIGNAL);
  return res;
}

int RS485::getc()
{
  unsigned char c;
  while(true){
    if (_recv_buf.read(&c)) break;
    Thread::wait(1);
  }
  return (int)c;
}

int RS485::printf(const char* format, ...)
{
  char str[256];
  va_list arg;
  va_start(arg, format);
  vsprintf(str, format, arg);
  va_end(arg);
  write(str, strlen(str));
  return 0;
}


ssize_t RS485::write(const void* buffer, size_t length)
{
  unsigned char *buf = (unsigned char *)buffer;
  int res;
  for(res = 0; res < length; res ++){
    if (!_send_buf.write(*buf ++)) break;
  }
  _send_thread.signal_set(SEND_START_SIGNAL);
  return res;
}

ssize_t RS485::read(void* buffer, size_t length)
{
  unsigned char *buf = (unsigned char *)buffer;
  int res;
  for(res = 0; res < length; res ++) {
    if (!_recv_buf.read(buf)) break;
  }
  return res;
}

void RS485::recvThreadStarter(void const *argument)
{
  RS485 *instance = (RS485*)argument;
  instance->recvThread();
}

void RS485::recvThread()
{
  while(true) {
    char c = _serial.getc();
    if ((_select_out == 1) || (_send_timer.read_us() < _guard_time_us)) continue;
    _recv_buf.write(c);
    _recv_timer.reset();
    _recv_timer.start();
  }
}

void RS485::sendThreadStarter(void const *argument)
{
  RS485 *instance = (RS485*)argument;
  instance->sendThread();
}

void RS485::sendThread()
{
  while(true){
    Thread::signal_wait(SEND_START_SIGNAL);
    while(_send_buf.size() > 0) {
      if (_recv_timer.read_us() < _guard_time_us){
        Thread::wait(0);
        continue;
      }
      _send_timer.reset();
      _send_timer.start();
      _select_out = 1;
      unsigned char c;
      _send_buf.read(&c);
      _serial.putc(c);
    }
    wait_us(_guard_time_us);
    _send_timer.reset();
    _send_timer.start();
    _select_out = 0;
  }
}

void RS485::txFinishCallback(void)
{
  _send_thread.signal_set(SEND_END_SIGNAL);
}
