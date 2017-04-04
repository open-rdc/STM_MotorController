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
  property_size[B3M_SYSTEM_BAUDRATE] = sizeof(long);
  property_size[B3M_SYSTEM_MCU_TEMP_LIMIT_PR] = sizeof(char);
  property_size[B3M_SYSTEM_CURRENT_LIMIT_PR] = sizeof(char);
  property_size[B3M_SYSTEM_LOCKDETECT_TIME] = sizeof(char);
  property_size[B3M_SYSTEM_LOCKDETECT_OUTRATE] = sizeof(char);
  property_size[B3M_SYSTEM_LOCKDETECT_TIME_PR] = sizeof(char);
  property_size[B3M_SYSTEM_TORQUE_LIMIT] = sizeof(char);  
  property_size[B3M_CONTROL_KP0] = sizeof(long);
  property_size[B3M_CONTROL_KD0] = sizeof(long);
  property_size[B3M_CONTROL_KI0] = sizeof(long);
  property_size[B3M_CONTROL_KP1] = sizeof(long);
  property_size[B3M_CONTROL_KD1] = sizeof(long);
  property_size[B3M_CONTROL_KI1] = sizeof(long);
  property_size[B3M_CONTROL_KP2] = sizeof(long);
  property_size[B3M_CONTROL_KD2] = sizeof(long);
  property_size[B3M_CONTROL_KI2] = sizeof(long);
  property_size[B3M_CONFIG_MODEL_NUMBER] = sizeof(long);
  property_size[B3M_CONFIG_MODEL_TYPE] = sizeof(long);
  property_size[B3M_CONFIG_FW_VERSION] = sizeof(long);
}

/*!
 * @brief parse command
 *
 * @param[in] command_data          additional command data
 * @param[in] command_data_len     length of additional command data
 * 
 * [READ]
 * > command data
 * size command option id address length sum
 * > reply status data (only single mode)
 * size command status id data(1) data(2) ... sum
 *
 * [WRITE]
 * > command data
 * size command option id1 data(1) data(2) ... id2 data(1) data(2) ... address count sum
 * > reply status data (only single mode)
 * size command status id sum
 *
 * problem about noise
 * http://www.softech.co.jp/mm_101102_firm.htm
 */ 
int Parser::setCommand(unsigned char *command_data, int command_data_len)
{
  int res = 0;
  if (command_data_len < 5) return res;
  command_buf_len = 0;
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
        res = B3M_CMD_WRITE;
        int count = command_buf[length - 2];
        int len = (length - 6) / count;
        int add = command_buf[length - 3];
        for(int i = 0; i < count; i ++){
          int index = 3 + i * len;
          if (command_buf[index] != property.ID) continue;
          unsigned char *p = (unsigned char *)&command_buf[index + 1];
          for(int j = 0; j < (len - 1);){
            int address = add + j;
            address_[stocked_data_len] = address;
            if (property_size[address] == sizeof(char)){
              data_[stocked_data_len] = (short)p[j];
              j ++;
            } else if (property_size[address] == sizeof(short)){
              data_[stocked_data_len] = (short)((unsigned short)(p[j+1] << 8) + (unsigned short)p[j]);
              j += 2;
            } else if (property_size[address] == sizeof(long)){
              data_[stocked_data_len] = (short)((unsigned short)(p[j+3] << 24)+
                (unsigned short)(p[j+2] << 16) + (unsigned short)(p[j+1] << 8) + (unsigned short)p[j]);
              j += 4;
            }
            if (stocked_data_len < (MAX_STOCKED_COMMAND - 1))
              stocked_data_len ++;
          }          
          if (count == 1){
            reply_byte = 5;
            reply[0] = 5, reply[1] = 0x84, reply[2] = 0, reply[3] = property.ID;
            reply[4] = 0;
            for(int i = 0; i < 4; i ++) reply[4] += reply[i];
          }
        }
      } else if (command == B3M_CMD_READ){
        if (command_buf[3] != property.ID) break;
        res = B3M_CMD_READ;
        int data_byte = command_buf[length - 2];
        int add = command_buf[length - 3];
        reply_byte = data_byte+5;
        reply[0] = reply_byte, reply[1] = 0x83, reply[2] = 0, reply[3] = property.ID;
        unsigned char *p = (unsigned char *)&property;
        for(int i = 0; i < data_byte; i ++) reply[i + 4] = p[add + i];
        reply[reply_byte - 1]  = 0;
        for(int i = 0; i < (reply_byte - 1); i ++) reply[reply_byte - 1] += reply[i];
      } else if (command == B3M_CMD_SAVE){
        if (command_buf[3] != property.ID) break;
        res = B3M_CMD_SAVE;
        reply_byte = 5;
        reply[0] = 5, reply[1] = 0x82, reply[2] = 0, reply[3] = property.ID;
        reply[4] = 0;
        for(int i = 0; i < 4; i ++) reply[4] += reply[i];
      } else if (command == B3M_CMD_LOAD){
        if (command_buf[3] != property.ID) break;
        res = B3M_CMD_LOAD;
        reply_byte = 5;
        reply[0] = 5, reply[1] = 0x81, reply[2] = 0, reply[3] = property.ID;
        reply[4] = 0;
        for(int i = 0; i < 4; i ++) reply[4] += reply[i];
      } else if (command == B3M_CMD_RESET){
        res = B3M_CMD_RESET;
      } else if (command == B3M_CMD_DATA_STOCK){
        res = B3M_CMD_DATA_STOCK;
      } else if (command == B3M_CMD_DATA_PLAY){
        if (command_buf[3] != property.ID) break;
        res = B3M_CMD_DATA_PLAY;
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
    *address = address_[0];
    *data = data_[0];
    for(int i = 1; i < stocked_data_len; i ++){
      address_[i - 1] = address_[i];
      data_[i - 1] = data_[i];
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
