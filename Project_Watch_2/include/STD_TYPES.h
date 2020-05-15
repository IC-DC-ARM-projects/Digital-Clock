/**
 * @file  STD_TYPES.h
 * @brief This file is to be used as an implementation of the standard types.
 *
 * @author MSN
 * @date Mar 31, 2020
 *
 */

#ifndef STD_TYPES_H
#define STD_TYPES_H

typedef  unsigned char           u8 ;
typedef  unsigned short int      u16;
typedef  unsigned long int       u32;
typedef  unsigned long long      u64;

typedef  signed char             s8 ;
typedef  signed short int        s16;
typedef  signed long int         s32;
typedef  signed long long        s64;

typedef  float                   f32;
typedef  double                  f64;
typedef  long double             f96;


typedef u8          Std_ReturnType;

#define E_OK        (Std_ReturnType)0
#define E_NOT_OK    (Std_ReturnType)1
#define E_BUSY      (Std_ReturnType)2

#define NULL ((void *)0)

#endif

