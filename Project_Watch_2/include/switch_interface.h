/**
 * @file  switch_interface.h
 * @brief This file is to be used as an implementation of the switch driver.
 *
 * @author MSN
 * @date Mar 31, 2020
 *
 */

#ifndef SWITCH_INTERFACE_H_
#define SWITCH_INTERFACE_H_

#define SWITCH_PRESSED   1
#define SWITCH_UNPRESSED 0


#define SWITCH_ACTIVE_LOW  0
#define SWITCH_ACTIVE_HIGH 1

/**
 * @brief Initialize all the switches: pin direction, default pull up/down
 * @param void
 * @return void
 */
void Switch_Init(void);


/**
 * @brief Return the state of the switch
 * @param switchNum: Switch number, this should be <= the amount of configured switches
 * @return the state of the switch:\n
 *         SWITCH_PRESSED,\n
 *         SWITCH_UNPRESSED
 */
u8 Switch_GetReading(u8 switchNum);

#endif /* SWITCH_INTERFACE_H_ */
