/*
 * hal_vector.h
 *
 *  Created on: Mar 2, 2021
 *      Author: keyvan
 */
#ifndef TYPES_H
#define TYPES_H


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Boolean true value. */
#define BL_TRUE       	(1)
/** \brief Boolean false value. */
#define BL_FALSE      	(0)
/** \brief NULL pointer value. */
#define NULL_PTR       	((void *)0)


/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef unsigned char   bool_t;                     /**<  boolean type               */
typedef char            char_t;                     /**<  character type             */
typedef unsigned long   addr_t;                     /**<  memory address type        */
typedef unsigned char   int8u_t;                    /**<  8-bit unsigned integer     */
typedef signed char     int8s_t;                    /**<  8-bit   signed integer     */
typedef unsigned short  int16u_t;                   /**< 16-bit unsigned integer     */
typedef signed short    int16s_t;                   /**< 16-bit   signed integer     */
typedef unsigned int    int32u_t;                   /**< 32-bit unsigned integer     */
typedef signed int      int32s_t;                   /**< 32-bit   signed integer     */

#endif /* TYPES_H */
/*********************************** end of types.h ************************************/
