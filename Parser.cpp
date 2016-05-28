/**
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 */

#include "b3m.h"
#include "Parser.h"

#include "mbed.h"
#include "RS485.h"
extern RS485 rs485;
extern DigitalOut led2;
extern DigitalOut led3;
extern DigitalOut led4;

/*!
 * @brief constructor
 */
 
 
Parser::Parser(int id): command_buf_len(0), stocked_data_len(0)
{
  setID(id);
}

// ‚Æ‚è‚ ‚¦‚¸‚ÌŽÀ‘•
int Parser::setCommand(unsigned char *command_data, int command_data_len)
{
  int res = 0;
  if (command_data_len <= 0) return false;
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
          res = B3M_CMD_WRITE;
//          int count = command_buf[length - 2];
//          int len = (length - 6) / count - 1;
          address[stocked_data_len] = command_buf[length - 3];
          unsigned char *p = (unsigned char *)&command_buf[4];
          data[stocked_data_len] = (short)((*(p+1)  << 8) + *p);
          if (stocked_data_len < MAX_STOCKED_COMMAND - 1)
            stocked_data_len ++;
          break;
        } else if (command == B3M_CMD_SAVE){
          res = B3M_CMD_SAVE;
        }
      }
      break;
    }
    for(int i = length; i < command_buf_len; i ++)
    {
      command_buf[i - length] = command_buf[i];
    }
    command_buf_len -= length;
  }
  return res;
}

void Parser::setId(int id)
{
  this->id = id;
}

int Parser::getId()
{
  return this->id;
}

int Parser::getNextCommand(int *address, int *data)
{
  int res = stocked_data_len;
  if (stocked_data_len > 0){
    *address = this->address[0];
    *data = this->data[0];
    for(int i = stocked_data_len - 1; i > 0; i --){
      address[i - 1] = address[i];
      data[i - 1] = data[i];
    }
    stocked_data_len --;
  }
  return res;
}
