/*
 * hal_vector.c
 *
 *  Created on: Mar 2, 2021
 *      Author: keyvan Hadjari
 */

#include "types.h"

extern void     Rs232Init(void);
extern void     Rs232TransmitByte(blt_int8u data);
extern void     Rs232TransmitPacket(blt_int8u *data, blt_int8u len);
extern blt_bool Rs232ReceivePacket(blt_int8u *data, blt_int8u *len);

const void* volatile _hal_vector_[] __attribute__((section(".hal_vector"))) = {
  // RS232 UART6,115200,RX(PC6),TX(PC7)
  Rs232Init,
  Rs232TransmitByte,
  Rs232TransmitPacket,
  Rs232ReceivePacket
};

