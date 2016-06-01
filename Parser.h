// Chiba Institute of Technology

#ifndef PARSER_H
#define PARSER_H

#include "mbed.h"

#define MAX_REPLY 256

/** Commnad Parser
 */

class Parser
{
  static const int MAX_COMMAND_LEN = 256;
  static const int MAX_STOCKED_COMMAND = 32;
public:
  Parser();

  int setCommand(unsigned char *command_data, int command_len);
  
  int getReply(unsigned char *data);

  void setId(int id);

  int getNextCommand(int *address, int *data);


private:
  unsigned char command_buf[MAX_COMMAND_LEN];
  int command_buf_len;
  int command;
  int option;
  int address_[MAX_STOCKED_COMMAND];
  int data_[MAX_STOCKED_COMMAND];
  int stocked_data_len;
  unsigned char reply[MAX_REPLY];
  int reply_byte;
};

#endif
