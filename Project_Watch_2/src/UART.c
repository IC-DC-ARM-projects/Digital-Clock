/*
 * uart.c
 *
 *  Created on: May 5, 2020
 *      Author: SARA
 */
#include "STD_TYPES.h"
#include "uart.h"
#include "uart_cfg.h"

typedef struct
{
	u32 USART_SR;
	u32 USART_DR;
	u32 USART_BRR;
	u32 USART_CR1;
	u32 USART_CR2;
	u32 USART_CR3;
	u32 USART_GTPR;
} UART_Reg_t;

#define UART_COUNT                           3

static volatile UART_Reg_t* const UART[UART_COUNT] = {
	(volatile UART_Reg_t *)0x40013800,
	(volatile UART_Reg_t *)0x40004400,
	(volatile UART_Reg_t *)0x40004800,
};

#define BREAK_FLAG                           0x00000100
#define TXE_FLAG                             0x00000080
#define TC_FLAG                              0x00000040
#define RXNE_FLAG                            0x00000020
#define OVR_FLAG                             0x00000008
#define NOISE_FLAG                           0x00000004
#define FRAME_FLAG                           0x00000002
#define PARITY_FLAG                          0x00000001
#define DMA_ERROR_FLAG                       0X00000001
#define PARITY_SEL_MASK                      0xFFFFFDFF
#define STOP_BIT_MASK                        0xFFFFFCFF
#define LBCL_MASK                            0xFFFFFFDF
#define UE_MASK                              0xFFFFDFFF
#define DATA_LENGTH_MASK                     0xFFFFEFFF
#define PARITY_MASK                          0xFFFFFBFF
#define PARITY_INT_MASK                      0xFFFFFEFF
#define TXEIE_MASK                           0xFFFFFF7F
#define TCIE_MASK                            0xFFFFFFBF
#define RXNEIE_MASK                          0xFFFFFFDF
#define TE_MASK                              0xFFFFFFF7
#define RE_MASK                              0xFFFFFFFB
#define LINEN_MASK                           0xFFFFBFFF
#define LIN_BREAK_INT_MASK                   0xFFFFFFBF
#define DMAT_MASK                            0xFFFFFF7F
#define DMAR_MASK                            0xFFFFFFBF
#define DMA_ERROR_MASK                       0xFFFFFFFE
#define SBK_SEND_MASK                        0x00000001
#define TC_MASK                              0xFFFFFFBF
#define BREAK_MASK                           0xFFFFFEFF
#define EIE_MASK                             0xFFFFFFFE
#define RXNEIE_FLAG                          0x00000020

#define CALLBACK_COUNT                       8
#define TX_BUFFER_EMPTY_CALLBACK             0
#define RX_BUFFER_FULL_CALLBACK              1
#define PARITY_CALLBACK                      2
#define TRANSMISSION_COMPLETE_CALLBACK       3
#define LINBREAK_CALLBACK                    4
#define OVR_ERROR_CALLBACK                   5
#define NOISE_ERROR_CALLBACK                 6
#define FRAME_ERROR_CALLBACK                 7


typedef void (*UART_Callback_t)(void);

static UART_Callback_t UART_Callback[UART_COUNT][CALLBACK_COUNT];

static void  UART_CentralizedIRQHandler(u8 peri);

void UART_Init(u8 peri, u32 baudRate)
{
	u8 i;
	u32 reg;

	/* clear flags early, because enabling peripheral clock raises some flags as TC */
	reg = UART[peri]->USART_SR;
	reg = UART[peri]->USART_DR;
	UART[peri]->USART_SR = 0;

	reg = UART[peri]->USART_CR1;

	/* Enable UART UE bit  */
	reg |= UART_STATE_CONTROL_ON;

	/* set M bit to 8 bit data */
	reg &= DATA_LENGTH_MASK;

	/* Disable Parity */
	reg &= PARITY_MASK;

	UART[peri]->USART_CR1 = reg;

	reg = UART[peri]->USART_CR2;

	/* Disable LIN */
	reg &= LINEN_MASK;

	/* configure 1 stop bit */
	reg &= STOP_BIT_MASK;

	UART[peri]->USART_CR2 = reg;

	reg = UART[peri]->USART_CR3;

	/* Disable DMAT */
	reg &= DMAT_MASK;

	/* Disable DMAR */
	reg &= DMAR_MASK;

	UART[peri]->USART_CR3 = reg;

	/* set baud rate */
	switch(peri)
	{
	case UART_PERI_1:
		UART[peri]->USART_BRR = (2 * UART_APB2_BUS_FREQ + baudRate) / (2 * baudRate);
	break;

	case UART_PERI_2:
	case UART_PERI_3:
		UART[peri]->USART_BRR = (2 * UART_APB1_BUS_FREQ + baudRate) / (2 * baudRate);
	break;
	}

	/* enable Tx & Rx */
	reg = UART[peri]->USART_CR1;

	reg |= UART_TANSMITTER_STATE_ON | UART_RECEIVER_STATE_ON;

	UART[peri]->USART_CR1 = reg;

	/* trap until IDLE frame is sent */
	while ((UART[peri]->USART_SR & TC_FLAG) == 0)
	{

	}

	/* clear flags */
	reg = UART[peri]->USART_SR;
	reg = UART[peri]->USART_DR;
	UART[peri]->USART_SR = 0;

	/* rest callbacks */
	for(i = 0; i < CALLBACK_COUNT; i++)
	{
		UART_Callback[peri][i] = 0;
	}
}

u8 UART_GetIsLIN_BreakDetection(u8 peri)
{
	return (!! (UART[peri]->USART_SR & BREAK_FLAG));
}

u8 UART_GetIsTxBufferEmpty(u8 peri)
{
	return (!! (UART[peri]->USART_SR & TXE_FLAG));
}

u8 UART_GetIsTansmitComplete(u8 peri)
{
	return (!! (UART[peri]->USART_SR & TC_FLAG));
}

u8 UART_GetIsRxBufferFull(u8 peri)
{
	return (!! (UART[peri]->USART_SR & RXNE_FLAG));
}

u8 UART_GetIsOverRunError(u8 peri)
{
	return (!! (UART[peri]->USART_SR & OVR_FLAG));

}

u8 UART_GetINoiseError(u8 peri)
{
	return (!! (UART[peri]->USART_SR & NOISE_FLAG));
}

u8 UART_GetIsFrameError(u8 peri)
{
	return (!! (UART[peri]->USART_SR & FRAME_FLAG));
}

u8 UART_GetIsParityError(u8 peri)
{
	return (!! (UART[peri]->USART_SR & PARITY_FLAG));
}

u16 UART_GetData(u8 peri)
{
	return UART[peri]->USART_DR;
}

void UART_ConfigData(u8 peri, u16 data)
{
	UART[peri]->USART_DR = data;
}

void UART_ConfigBaudRate(u8 peri, u16 baud_rate)
{
	u32 fclk;

	switch(peri)
	{
	case UART_PERI_1:
		fclk = UART_APB2_BUS_FREQ;
	break;

	case UART_PERI_2:
	case UART_PERI_3:
		fclk = UART_APB1_BUS_FREQ;
	break;
	}

	UART[peri]->USART_BRR = (2 * fclk + baud_rate) / (2 * baud_rate);
}

void UART_ConfigParityType(u8 peri, u32 parity_select)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= PARITY_SEL_MASK;
	reg |= parity_select;
	UART[peri]->USART_CR1 = reg;
}

void UART_ConfigStopBits(u8 peri, u32 stop_bits)
{
	u32 reg = UART[peri]->USART_CR2;
	reg &= STOP_BIT_MASK;
	reg |= stop_bits;
	UART[peri]->USART_CR2 = reg;
}

void UART_ConfigLINBreakDetectionLength(u8 peri, u32 lin_break_detection_length)
{
	u32 reg = UART[peri]->USART_CR2;
	reg &= LBCL_MASK;
	reg |= lin_break_detection_length;
	UART[peri]->USART_CR2 = reg;
}

void UART_ControlState(u8 peri, u32 state_control)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= UE_MASK;
	reg |= state_control;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlDataLength(u8 peri, u32 data_length)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= DATA_LENGTH_MASK;
	reg |= data_length;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlParityState(u8 peri, u32 parity_state)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= PARITY_MASK;
	reg |= parity_state;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlParityErrorINT(u8 peri, u32 parity_control_INT)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= PARITY_INT_MASK;
	reg |= parity_control_INT;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlTxBufferEmptyINT(u8 peri, u32 tx_buffer_empty_control_INT)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= TXEIE_MASK;
	reg |= tx_buffer_empty_control_INT;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlTransmissionCompleteINT(u8 peri, u32 transmission_complete_control_INT)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= TCIE_MASK;
	reg |= transmission_complete_control_INT;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlRxBufferFullINT(u8 peri, u32 rx_buffer_full_control_INT)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= RXNEIE_MASK;
	reg |= rx_buffer_full_control_INT;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlTransmitterState(u8 peri, u32 transmit_state)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= TE_MASK;
	reg |= transmit_state;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlReceiverState(u8 peri, u32 receive_state)
{
	u32 reg = UART[peri]->USART_CR1;
	reg &= RE_MASK;
	reg |= receive_state;
	UART[peri]->USART_CR1 = reg;
}

void UART_ControlLINState(u8 peri, u32 lin_state_control)
{
	u32 reg = UART[peri]->USART_CR2;
	reg &= LINEN_MASK;
	reg |= lin_state_control;
	UART[peri]->USART_CR2 = reg;
}

void UART_ControlLINBreakDetectionINT(u8 peri, u32 lin_break_detection_control_INT)
{
	u32 reg = UART[peri]->USART_CR2;
	reg &= LIN_BREAK_INT_MASK;
	reg |= lin_break_detection_control_INT;
	UART[peri]->USART_CR2 = reg;
}

void UART_ControlDMATransmitterState(u8 peri, u32 dma_transmitter_state)
{
	u32 reg = UART[peri]->USART_CR3;
	reg &= DMAT_MASK;
	reg |= dma_transmitter_state;
	UART[peri]->USART_CR3 = reg;
}

void UART_ControlDMAReceiverState(u8 peri, u32 dma_receivier_state)
{
	u32 reg = UART[peri]->USART_CR3;
	reg &= DMAR_MASK;
	reg |= dma_receivier_state;
	UART[peri]->USART_CR3 = reg;
}

void UART_ControlDMAErrorINT(u8 peri, u32 dma_error_control_INT)
{
	u32 reg = UART[peri]->USART_CR3;
	reg &= DMA_ERROR_MASK;
	reg |= dma_error_control_INT;
	UART[peri]->USART_CR3 = reg;
}

void UART_SetBreakCharacter(u8 peri)
{
	u32 reg = UART[peri]->USART_CR1;
	reg |= SBK_SEND_MASK;
	UART[peri]->USART_CR1 = reg;
}

void UART_ConfigTxBufferEmptyCallback(u8 peri, UART_TxBufferEmptyCallback_t ptr)
{
	UART_Callback[peri][TX_BUFFER_EMPTY_CALLBACK] = (UART_Callback_t)ptr;
}

void  UART_ConfigRxBufferFullCallback(u8 peri, UART_RxBufferFullCallback_t ptr)
{
	UART_Callback[peri][RX_BUFFER_FULL_CALLBACK] = (UART_Callback_t)ptr;
}

void UART_ConfigParityErrorCallback(u8 peri, UART_ParityErrorCallback_t ptr)
{
	UART_Callback[peri][PARITY_CALLBACK] = (UART_Callback_t)ptr;
}

void UART_ConfigNoiseErrorCallBack(u8 peri, UART_NoiseErrorCallback_t ptr)
{
	UART_Callback[peri][NOISE_ERROR_CALLBACK] = (UART_Callback_t)ptr;
}

void UART_ConfigOverRunErrorCallBack(u8 peri, UART_OverRunErrorCallback_t ptr)
{
	UART_Callback[peri][OVR_ERROR_CALLBACK] = (UART_Callback_t)ptr;
}

void UART_ConfigTransmissionCompleteCallback(u8 peri, UART_TranssmisonCompleteCallback_t ptr)
{
	UART_Callback[peri][TRANSMISSION_COMPLETE_CALLBACK] = (UART_Callback_t)ptr;
}

void UART_ConfigFrameErrorCallBack(u8 peri, UART_ErrorFrameCallback_t ptr)
{
	UART_Callback[peri][FRAME_ERROR_CALLBACK] = (UART_Callback_t)ptr;
}

void UART_ConfigLINBreakCallback(u8 peri, UART_LINBreakCallback_t ptr)
{
	UART_Callback[peri][LINBREAK_CALLBACK] = (UART_Callback_t)ptr;
}

static void UART_CentralizedIRQHandler(u8 peri)
{
	register u32 dr_reg;
	register u32 sr_reg;
	register u32 cr1_reg;
	register u32 cr3_reg;

	sr_reg = UART[peri]->USART_SR;
	cr1_reg = UART[peri]->USART_CR1;
	cr3_reg = UART[peri]->USART_CR3;

	if(sr_reg & PARITY_FLAG)
	{
		if(sr_reg & RXNE_FLAG)
		{
			sr_reg = UART[peri]->USART_SR;
			dr_reg = UART[peri]->USART_DR;

			if (cr1_reg & ~PARITY_INT_MASK)
			{
				if(UART_Callback[peri][PARITY_CALLBACK])
				{
					((UART_ParityErrorCallback_t)(UART_Callback[peri][PARITY_CALLBACK]))();
				}
				else
				{

				}
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & OVR_FLAG)
	{
		sr_reg = UART[peri]->USART_SR;
		dr_reg = UART[peri]->USART_DR;

		if((cr1_reg & RXNEIE_FLAG) || ((cr3_reg & ~DMAR_MASK) && (cr3_reg & ~EIE_MASK)))
		{
			if(UART_Callback[peri][OVR_ERROR_CALLBACK])
			{
				((UART_OverRunErrorCallback_t)(UART_Callback[peri][OVR_ERROR_CALLBACK]))();
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & FRAME_FLAG)
	{
		sr_reg = UART[peri]->USART_SR;
		dr_reg = UART[peri]->USART_DR;

		if((cr1_reg & RXNEIE_FLAG) || ((cr3_reg & ~DMAR_MASK) && (cr3_reg & ~EIE_MASK)))
		{
			if(UART_Callback[peri][FRAME_ERROR_CALLBACK])
			{
				((UART_ErrorFrameCallback_t)(UART_Callback[peri][FRAME_ERROR_CALLBACK]))();
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & NOISE_FLAG)
	{
		sr_reg = UART[peri]->USART_SR;
		dr_reg = UART[peri]->USART_DR;

		if((cr1_reg & RXNEIE_FLAG) || ((cr3_reg & ~DMAR_MASK) && (cr3_reg & ~EIE_MASK)))
		{
			if(UART_Callback[peri][NOISE_ERROR_CALLBACK])
			{
				((UART_NoiseErrorCallback_t)(UART_Callback[peri][NOISE_ERROR_CALLBACK]))();
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & TXE_FLAG)
	{
		if (cr1_reg & ~TXEIE_MASK)
		{
			if(UART_Callback[peri][TX_BUFFER_EMPTY_CALLBACK])
			{
				((UART_TxBufferEmptyCallback_t)(UART_Callback[peri][TX_BUFFER_EMPTY_CALLBACK]))();
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & RXNE_FLAG)
	{
		dr_reg = UART[peri]->USART_DR;

		if (cr1_reg & ~RXNEIE_MASK)
		{
			if(UART_Callback[peri][RX_BUFFER_FULL_CALLBACK])
			{
				((UART_RxBufferFullCallback_t)(UART_Callback[peri][RX_BUFFER_FULL_CALLBACK]))((u16)dr_reg);
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & TC_FLAG)
	{
		UART[peri]->USART_SR &= TC_MASK;

		if (cr1_reg & ~TCIE_MASK)
		{
			if(UART_Callback[peri][TRANSMISSION_COMPLETE_CALLBACK])
			{
				((UART_TxBufferEmptyCallback_t)(UART_Callback[peri][TRANSMISSION_COMPLETE_CALLBACK]))();
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}

	if(sr_reg & BREAK_FLAG)
	{
		UART[peri]->USART_SR &= BREAK_MASK;

		if (UART[peri]->USART_CR2 & ~LIN_BREAK_INT_MASK)
		{
			if(UART_Callback[peri][LINBREAK_CALLBACK])
			{
				((UART_LINBreakCallback_t)(UART_Callback[peri][LINBREAK_CALLBACK]))();
			}
			else
			{

			}
		}
		else
		{

		}
	}
	else
	{

	}
}

void USART1_IRQHandler(void)
{
	UART_CentralizedIRQHandler(UART_PERI_1);
}

void USART2_IRQHandler(void)
{
	UART_CentralizedIRQHandler(UART_PERI_2);
}

void USART3_IRQHandler(void)
{
	UART_CentralizedIRQHandler(UART_PERI_3);
}
