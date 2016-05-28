// Chiba Institute of Technology

#ifndef PARSER_H
#define PARSER_H

#include "mbed.h"

/** Commnad Parser
 */

class Parser
{
  static const int MAX_COMMAND_LEN = 256;
  static const int MAX_STOCKED_COMMAND = 10;
public:
  Parser();

  int setCommand(unsigned char *command_data, int command_len);
  
  void setId(int id);

  int getNextCommand(int *address, int *data);

private:
  unsigned char command_buf[MAX_COMMAND_LEN];
  int command_buf_len;
  int command;
  int option;
  int address[MAX_STOCKED_COMMAND];
  int data[MAX_STOCKED_COMMAND];
  int stocked_data_len;
};

#endif
