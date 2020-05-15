#include "STD_TYPES.h"
#include "DMA.h"

typedef struct
{
	/* Interrupt status register */
	const u32 DMA_ISR;

	/* Interrupt flag clear register */
	u32 DMA_IFCR;
} DMA_InterruptRegisters_t;

typedef struct
{
	/* Configuration register */
	u32 DMA_CCR;

	/* Number of data register */
	u32 DMA_CNDTR;

	/* Peripheral address register */
	u32 DMA_CPAR;

	/* Memory address register */
	u32 DMA_CMAR;
	const u32 _padding;
} DMA_ChannelRegisters_t;

typedef struct
{
	DMA_InterruptRegisters_t      interruptRegisters;
	DMA_ChannelRegisters_t        channelRegisters[7];
} DMA_t;

#define TEIF_MASK            8
#define HTIF_MASK            4
#define TCIF_MASK            2
#define GIF_MASK             1

#define CTEIF_MASK           8
#define CHTIF_MASK           4
#define CTCIF_MASK           2
#define CGIF_MASK            1

#define TEIE_MASK            0x00000008
#define HTIE_MASK            0x00000004
#define TCIE_MASK            0x00000002

#define EN_MASK              0xFFFFFFFE

#define CHANNEL_MUL_FACTOR   2

#define DMA_DEVICES_COUNT    2
#define CHANNELS_MAX_COUNT   7
#define CHANNEL_EVENTS_COUNT 3

/* Selector for the combined DMA channels 4 & 5 */
#define DMA_CHANNEL4_5       7


/* private/internal objects declarations/definitions */
/* Contains the base address of each DMA peripheral */
static volatile DMA_t* const DMA[2] = {
		(volatile DMA_t*)0x40020000, /* DMA1 base address */
		(volatile DMA_t*)0x40020400, /* DMA2 base address */
};

/* Callback for each each channel event, for all channels, per each DMA peripheral */
static DMA_Callback_t DMAcallback[DMA_DEVICES_COUNT][CHANNELS_MAX_COUNT][CHANNEL_EVENTS_COUNT];


/* private/internal functions declarations */
static void DMA_HandleIRQ(u8 peri, u32 channel);


/* initialization API */
void DMA_InitChannel(const DMA_ChannelCfg_t* cfg)
{
	u8 i;

	/* reset all registers */
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CCR   = 0x00000000;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CNDTR = 0x00000000;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CPAR  = 0x00000000;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CMAR  = 0x00000000;

	for (i = 0; i < CHANNEL_EVENTS_COUNT; i++) /* foreach event in the channel */
	{
		DMAcallback[cfg->peri][cfg->channel][i] = 0; /* reset the callback function */
	}

	/* reset all the DMA channels flags */
	DMA[cfg->peri]->interruptRegisters.DMA_IFCR = 0xFFFFFFFF;
}


/* getter APIs */
u8 DMA_GetIsErrorTransfer(const DMA_ChannelCfg_t* cfg)
{
	return !!(DMA[cfg->peri]->interruptRegisters.DMA_ISR & (TEIF_MASK << (cfg->channel << CHANNEL_MUL_FACTOR)));
}

u8 DMA_GetIsHalfTransfer(const DMA_ChannelCfg_t* cfg)
{
	return !!(DMA[cfg->peri]->interruptRegisters.DMA_ISR & (HTIF_MASK << (cfg->channel << CHANNEL_MUL_FACTOR)));
}

u8 DMA_GetIsTransferComplete(const DMA_ChannelCfg_t* cfg)
{
	return !!(DMA[cfg->peri]->interruptRegisters.DMA_ISR & (TCIF_MASK << (cfg->channel << CHANNEL_MUL_FACTOR)));
}

u8 DMA_GetIsGlobalInterrupt(const DMA_ChannelCfg_t* cfg)
{
	return !!(DMA[cfg->peri]->interruptRegisters.DMA_ISR & (GIF_MASK << (cfg->channel << CHANNEL_MUL_FACTOR)));
}


/* configuration API */
void DMA_ConfigChannel(const DMA_ChannelCfg_t* cfg)
{
	DMAcallback[cfg->peri][cfg->channel][DMA_CALLBACK_TYPE_TRANSFER_ERROR] = cfg->transfer_error_callback;
	DMAcallback[cfg->peri][cfg->channel][DMA_CALLBACK_TYPE_TRANSFER_COMPLETE] = cfg->transfer_complete_callback;
	DMAcallback[cfg->peri][cfg->channel][DMA_CALLBACK_TYPE_HALF_TRANSFER] = cfg->half_transfer_callback;

	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CNDTR = cfg->data_count;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CPAR = (u32)cfg->peripheral_address;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CMAR = (u32)cfg->memory_address;

	u32 reg = cfg->mem2mem_mode_control        |
			    cfg->channel_priority            |
			    cfg->memory_size                 |
			    cfg->peripheral_size             |
			    cfg->memory_inc_mode_control     |
			    cfg->peripheral_inc_mode_control |
			    cfg->circular_mode_control       |
			    cfg->data_direction              |
			    cfg->transfer_error_int_control  |
			    cfg->half_transfer_int_control   |
			    cfg->transfer_complete_int_control;
	reg |= DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CCR & ~EN_MASK;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CCR = reg;
}


/* control API */
void DMA_ControlChannel(const DMA_ChannelCfg_t* cfg, u32 channel_control)
{
	u32 reg = DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CCR;
	reg &= EN_MASK;
	reg |= channel_control;
	DMA[cfg->peri]->channelRegisters[cfg->channel].DMA_CCR = reg;
}


static void DMA_HandleIRQ(u8 peri, u32 channel)
{
	/* cache the volatile peripheral register DMA_ISR for speed */
	register u32 dma_isr_reg = (u32)DMA[peri]->interruptRegisters.DMA_ISR;

	register u32 channelBitsBlockStartLocation;

	if (channel == DMA_CHANNEL4_5) /* if the channel is the combined 4 & 5 */
	{
		/* if any of channel 4's events was triggered {half-transfer, transfer-complete, transfer-error} */
		if (dma_isr_reg & (CGIF_MASK << (DMA_CHANNEL4 << CHANNEL_MUL_FACTOR)))
		{
			/* set the channel selector to channel 4 */
			channel = DMA_CHANNEL4;
		}
		/* if any of channel 5's events was triggered {half-transfer, transfer-complete, transfer-error} */
		else if (dma_isr_reg & (CGIF_MASK << (DMA_CHANNEL5 << CHANNEL_MUL_FACTOR)))
		{
			/* set the channel selector to channel 4 */
			channel = DMA_CHANNEL5;
		}
	}
	else /* if the channel selector is anything other than the combined 4 & 5 */
	{

	}

	/* cache the expression that calculates the bit start location of the channel block */
	channelBitsBlockStartLocation = channel << CHANNEL_MUL_FACTOR;

	/* if channel event type is half-transfer */
	if (dma_isr_reg & (HTIF_MASK << (channelBitsBlockStartLocation)))
	{
		DMA[peri]->interruptRegisters.DMA_IFCR |= CHTIF_MASK << (channelBitsBlockStartLocation);

		if (DMA[peri]->channelRegisters[channel].DMA_CCR & HTIE_MASK)
		{
			if (DMAcallback[peri][channel][DMA_CALLBACK_TYPE_HALF_TRANSFER]) /* if the callback is not empty */
			{
				DMAcallback[peri][channel][DMA_CALLBACK_TYPE_HALF_TRANSFER]();
			}
			else /* if the callback is empty */
			{

			}
		}
		else
		{

		}
	}
	else /* if channel event half-transfer didn't occur */
	{

	}

	/* if channel event type is transfer-complete */
	if (dma_isr_reg & (TCIF_MASK << (channelBitsBlockStartLocation)))
	{
		DMA[peri]->interruptRegisters.DMA_IFCR |= CTCIF_MASK << (channelBitsBlockStartLocation);

		if (DMA[peri]->channelRegisters[channel].DMA_CCR & TCIE_MASK)
		{
			if(DMAcallback[peri][channel][DMA_CALLBACK_TYPE_TRANSFER_COMPLETE]) /* if the callback is not empty */
			{
				DMAcallback[peri][channel][DMA_CALLBACK_TYPE_TRANSFER_COMPLETE]();
			}
			else /* if the callback is empty */
			{

			}
		}
		else
		{

		}
	}
	else /* if channel event transfer-complete didn't occur */
	{

	}

	/* if channel event type is transfer-error */
	if (dma_isr_reg & (TEIF_MASK << (channelBitsBlockStartLocation)))
	{
		DMA[peri]->interruptRegisters.DMA_IFCR |= CTEIF_MASK << (channelBitsBlockStartLocation);

		if (DMA[peri]->channelRegisters[channel].DMA_CCR & TEIE_MASK)
		{
			if(DMAcallback[peri][channel][DMA_CALLBACK_TYPE_TRANSFER_ERROR]) /* if the callback is not empty */
			{
				DMAcallback[peri][channel][DMA_CALLBACK_TYPE_TRANSFER_ERROR]();
			}
			else /* if the callback is empty */
			{

			}
		}
		else
		{

		}
	}
	else /* if channel event transfer-error didn't occur */
	{

	}
}


/* DMA1 IRQs */
void DMA1_Channel1_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL1);
}

void DMA1_Channel2_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL2);
}

void DMA1_Channel3_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL3);
}

void DMA1_Channel4_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL4);
}

void DMA1_Channel5_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL5);
}

void DMA1_Channel6_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL6);
}

void DMA1_Channel7_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI1, DMA_CHANNEL7);
}


/* DMA2 IRQs */
void DMA2_Channel1_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI2, DMA_CHANNEL1);
}

void DMA2_Channel2_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI2, DMA_CHANNEL2);
}

void DMA2_Channel3_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI2, DMA_CHANNEL3);
}

void DMA2_Channel4_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI2, DMA_CHANNEL4);
}

void DMA2_Channel5_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI2, DMA_CHANNEL5);
}

void DMA2_Channel4_5_IRQHandler(void)
{
	DMA_HandleIRQ(DMA_PERI2, DMA_CHANNEL4_5);
}
