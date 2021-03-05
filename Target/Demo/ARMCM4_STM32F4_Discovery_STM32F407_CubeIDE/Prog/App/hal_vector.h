/*
 * hal_vector.h
 *
 *  Created on: Mar 2, 2021
 *      Author: keyvan
 */

#ifndef HAL_VECTOR_H_
#define HAL_VECTOR_H_

#include "types.h"

typedef void (*__void__void)();
typedef void (*__void__int8u)(int8u_t);
typedef void (*__void__int8up__int8u)(int8u_t*, int8u_t);
typedef int8u_t (*__int8u__int8up__int8up)(int8u_t*, int8u_t*);

#define HAL_VECTOR 	((volatile void*)0x08000188UL)

/*
hal_vector[0] = 0800422D
hal_vector[1] = 08004315
hal_vector[2] = 08004341
hal_vector[3] = 08004269
 */

#define Rs232Init() 				((__void__void)				((volatile void*)(*(((int32u_t*)HAL_VECTOR + 0)))))	()
#define Rs232TransmitByte(x) 		((__void__int8u)			((volatile void*)(*(((int32u_t*)HAL_VECTOR + 1)))))	(x)
#define Rs232TransmitPacket(x, y)	((__void__int8up__int8u)	((volatile void*)(*(((int32u_t*)HAL_VECTOR + 2)))))	(x, y)
#define Rs232ReceivePacket(x, y)	((__int8u__int8up__int8up)	((volatile void*)(*(((int32u_t*)HAL_VECTOR + 3)))))	(x, y)

#endif /* HAL_VECTOR_H_ */
