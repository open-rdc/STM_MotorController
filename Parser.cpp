/**
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 */

#include "b3m.h"
#include "Parser.h"

/*!
 * @brief constructor
 */
 
 
Parser::Parser(int id): command_buf_len(0), stocked_data_len(0)
{
  setID(id);
}

// ‚Æ‚è‚ ‚¦‚¸‚ÌŽÀ‘•
bool Parser::setCommand(unsigned char *command_data, int command_data_len)
{
  bool res = false;
  for(int i = 0; i < command_data_len; i ++)
    command_buf[command_buf_len ++] = command_data[i];
  int length = command_buf[0];
  if (command_buf_len >= length){
    while(1){
      if (command_buf[3] != id) break;
      {
        int sum = 0;
        for(int i = 0; i < length - 1; i ++){
          sum += command_buf[i];
        }
        if (command_buf[length - 1] != (sum & 0xff)) break;
        command = command_buf[1];
        option = command_buf[2];
        if (command == B3M_CMD_WRITE){
          int count = command_buf[length - 2];
          int len = (length - 6) / count - 1;
          address[stocked_data_len] = command_buf[length - 3];
          unsigned char *p = (unsigned char *)&command_buf[4];
          data[stocked_data_len] = (short)(*(p+1) + *p);
          stocked_data_len ++;
          break;
        }
      }
      break;
    }
    for(int i = length; i < command_buf_len; i ++)
    {
      command_buf[i - length] = command_buf[i];
    }
    command_buf_len -= length;
    res = true;
  }
  return res;
}
