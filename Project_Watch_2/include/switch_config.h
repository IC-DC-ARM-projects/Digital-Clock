/**
 * @file  switch_config.h
 * @brief This file is to be used as an implementation of the switch configuration.
 *
 * @author MSN
 * @date Mar 31, 2020
 *
 */

#ifndef SWITCH_CONFIG_H_
#define SWITCH_CONFIG_H_

#define SWITCH_COUNT 7


typedef struct
{
   GPIO_t switchIO;
   u8     activeState;
} Switch_cfg_t;


#endif /* SWITCH_CONFIG_H_ */
