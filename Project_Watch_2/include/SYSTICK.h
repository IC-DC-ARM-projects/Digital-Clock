/**
 * @file  SYSTICK.h
 * @brief This file is to be used as an implementation of the SysTick driver.
 *
 * @author MSN
 * @date Mar 31, 2020
 *
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#define SYSTICK_ENABLE                1U
#define SYSTICK_DISABLE               0U
#define SYSTICK_INTERRUPT_ENABLE      (u32)0x00000002


#define CLOCK_PRESCALER_AHB_DIV_8     (u32)0x00000000
#define CLOCK_PRESCALER_AHB_DIV_1     (u32)0x00000004

#define CLOCK_FREQUENCY_8_MHZ         (u32)8000000UL
#define CLOCK_FREQUENCY_8_MHZ_DIV8    (u32)1000000UL


/**
 * @brief Initialize SysTick
 * @param void
 * @return void
 */
extern void SYSTICK_Init(void);


/**
 * @brief To stop SysTick
 * @param void
 * @return void
 */
extern void SYSTICK_Stop(void);


/**
 * @brief To start SysTick
 * @param void
 * @return void
 */
extern void SYSTICK_Start(void);


/**
 * @brief Function to set tick time (ms)
 * @param Timer: time in ms
 * @return void
 */
extern void SYSTICK_SetTimers(u32 Timers);


/**
 * @brief Function callback to handle call back function
 * @param Copy_SystickCbf_t: (pointer to function)
 * @return E_OK     :- if the input argument is correct . \n
 *                     (if the input pointer to function is valid) \n
 *         E_NOT_OK :- if there's something wrong with the input argument . \n
 *                     (if the input pointer to function is not valid)
 */
Std_ReturnType SYSTICK_SetCallBack( void(*Copy_SystickCbf_t)(void));


#endif /* SYSTICK_H_ */
