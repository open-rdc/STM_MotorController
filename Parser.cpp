/**
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 */

#include "b3m.h"
#include "Parser.h"

Property property;

char property_size[sizeof(Property)];

/*!
 * @brief constructor
 */ 
Parser::Parser(): command_buf_len(0), stocked_data_len(0), reply_byte(0)
{
  for(int i = 0; i < sizeof(Property); i ++)
    property_size[i] = sizeof(short);
  property_size[B3M_SYSTEM_ID] = sizeof(char);
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
      int sum = 0;
      for(int i = 0; i < length - 1; i ++){
        sum += command_buf[i];
      }
      if (command_buf[length - 1] != (sum & 0xff)) break;
      command = command_buf[1];
      option = command_buf[2];
      if (command == B3M_CMD_WRITE){
        if (command_buf[3] != property.ID) break;
        res = B3M_CMD_WRITE;
//          int count = command_buf[length - 2];
//          int len = (length - 6) / count - 1;
        int add = command_buf[length - 3];
        address[stocked_data_len] = add;
        unsigned char *p = (unsigned char *)&command_buf[4];
        if (property_size[add] == sizeof(char))
          data[stocked_data_len] = (short)*p;
        if (property_size[add] == sizeof(short))
          data[stocked_data_len] = (short)((*(p+1)  << 8) + *p);
        if (stocked_data_len < MAX_STOCKED_COMMAND - 1)
          stocked_data_len ++;
        reply_byte = 5;
        reply[0] = 5, reply[1] = 0x84, reply[2] = 0, reply[3] = property.ID;
        reply[4] = 0;
        for(int i = 0; i < 4; i ++) reply[4] += reply[i];
      } else if (command == B3M_CMD_SAVE){
        if (command_buf[3] != property.ID) break;
        res = B3M_CMD_SAVE;
        reply_byte = 5;
        reply[0] = 5, reply[1] = 0x82, reply[2] = 0, reply[3] = property.ID;
        reply[4] = 0;
        for(int i = 0; i < 4; i ++) reply[4] += reply[i];
      } else if (command == B3M_CMD_RESET){
        res = B3M_CMD_RESET;
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

int Parser::getReply(unsigned char *data){
  int ret = reply_byte;
  for(int i = 0; i < reply_byte; i ++) data[i] = reply[i];
  reply_byte = 0;
  return ret;
}
