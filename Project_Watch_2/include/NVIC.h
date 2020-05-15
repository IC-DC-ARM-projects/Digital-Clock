/**
 * @file  NVIC.h
 * @brief This file is to be used as an implementation of the NVIC driver.
 *
 * @author MSN
 * @date Mar 31, 2020
 *
 */

#ifndef NVIC_H
#define NVIC_H

#define EXTI0_IRQNUMBER          6U
#define EXTI1_IRQNUMBER          7U
#define EXTI2_IRQNUMBER          8U
#define EXTI3_IRQNUMBER          9U
#define EXTI4_IRQNUMBER          10U
#define USART1_IRQNUMBER         37U
#define USART2_IRQNUMBER         38U
#define USART3_IRQNUMBER         39U
#define UART4_IRQNUMBER          52U
#define UART5_IRQNUMBER          53U
#define DMA1_Channel4_IRQHandler 14U
#define DMA1_Channel5_IRQHandler 15U


/**
 * @brief Function to enable interrupt
 * @param IRQNumber: interrupt request number from 0 to 80
 * @return Std_ReturnType: \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_EnableIRQ(u8 IRQNumber);


/**
 * @brief Function to disable interrupt
 * @param IRQNumber: interrupt request number from 0 to 80
 * @return Std_ReturnType \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_DisableIRQ(u8 IRQNumber);


/**
 * @brief Function to changes interrupt state to pending
 * @param IRQNumber: interrupt request number from 0 to 80
 * @return Std_ReturnType \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_SetPendingIRQ(u8 IRQNumber);


/**
 * @brief Function to removes the pending state of an interrupt
 * @param IRQNumber: interrupt request number from 0 to 80
 * @return Std_ReturnType \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_ClearPendingIRQ(u8 IRQNumber);


/**
 * @brief Function return status if the corresponding interrupt is active or not.
 * @param IRQNumber: interrupt request number from 0 to 80
 * @return Std_ReturnType \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_ISActive(u8 IRQNumber);


/**
 * @brief Function return status if the corresponding interrupt is active or not.
 * @param IRQNumber: interrupt request number from 0 to 80
 * @param Priority: interrupt priority number
 * @return Std_ReturnType \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_SetPriority (u8 IRQNumber, u8 Priority);


/**
 * @brief Function to generate interrupt software
 * @param IRQNumber: interrupt request number from 0 to 80
 * @return Std_ReturnType \n
 *         E_OK: request accepted \n
			  E_NOT_OK: request not accepted
 */
Std_ReturnType NVIC_SoftwareInterrupt(u8 IRQNumber);


/**
 * @brief Function to enable IRQ interrupts.
 * @param void
 * @return void
 */
void NVIC_EnableAllInterrupt(void);


/**
 * @brief Function to disable IRQ interrupts.
 * @param void
 * @return void
 */
void NVIC_DisableAllInterrupt(void);

/**
 * @brief Function to disable all fault exceptions.
 * @param void
 * @return void
 */
void NVIC_DisableAllFaults(void);

/**
 * @brief Set priority group
 * @param priority_grouping: priority group
 * @return void
 */
void NVIC_SetPriorityGrouping(u32 priority_grouping);


#endif
