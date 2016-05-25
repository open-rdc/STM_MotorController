// Chiba Institute of Technology

#ifndef PARSER_H
#define PARSER_H

#include "mbed.h"

/** Commnad Parser
 */

class Parser
{
  static const int MAX_COMMAND_LEN = 256;
public:
  Parser(int id);

  void setID(int id){
    this->id = id;
  }

  bool setCommand(unsigned char *command_data, int command_len);

  int getCommand();

  int getId();

  int getNextAddress();

  int getNextData();

private:
  unsigned char* command_buf;
  int command_buf_len;
  int id;
  int command;
  int option;
  int address[10];
  int data[10];
  int stocked_data_len;
};

#endif
