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
 *   while(1) {
 *     rs485.printf("Hello World\r\n");
 *     wait(0.5);
 *   }
 * }
 */

#include "mbed.h"
#include "rtos.h"
#include "RingBuffer.h"

#define MAX_RECV_BUFFER 256
#define MAX_SEND_BUFFER 32

class RS485
{
public:
  RS485(PinName tx, PinName rx, PinName selectOut);

  void baud(int baudrate);

  void guardTime(int guard_time_us);
    
  int readable();

  bool isEnableSend();

  int putc(int c);

  int getc();

  int printf(const char* format, ...);

  virtual ssize_t write(const void* buffer, size_t length);
  virtual ssize_t read(void* buffer, size_t length);

private:
  RawSerial _serial;
  DigitalOut _select_out;     // 0:input, 1:output
  float _guard_time_us;
  void txFinishCallback(void);

  Timer _send_timer, _recv_timer;

  static void recvThreadStarter(void const *argument);
  void recvThread(void);
  Thread _recv_thread;
  RingBuffer _recv_buf;

  static void sendThreadStarter(void const *argument);
  void sendThread(void);
  Thread _send_thread;
  RingBuffer _send_buf;
};

#endif

