/* libs */
#include "STD_TYPES.h"
/* MCAL */
#include "RCC.h"
#include "UART.h"
#include "GPIO.h"
#include "NVIC.h"
#include "DMA.h"
/* HAL */
#include "LCD.h"
#include "switch_interface.h"
/* Services */
#include "sched_interface.h"
#include <diag/Trace.h>

/* 23:59:59
 * 59 + 59*60 + 23*60*60 */
#define MAX_SECONDS                ((u32)86399)

/* periodicity of the app task in ms */
#define APP_TASK_PERIODICITY_MS    (100)

/* 1 min   = 1*60 sec
 * +1 min  = +60 sec
 * -1 min  = -60 sec
 * -10 min = -600 sec
 *
 * 1 hr   = 1*60*60 sec
 * +1 hr  = +60*60 sec
 * -1 hr  = -60*60 sec
 * -10 hr = -10*60*60 sec
 * */
/* 13:13:43 */

/* 1 second in miilisec */
#define ONE_SECOND_AS_MILLISEC     ((u16)1000)

/* 1 minute in seconds */
#define ONE_MINUTE_AS_SECONDS      ((u16)60)

/* 10 minutes in seconds */
#define TEN_MINUTES_AS_SECONDS     ((u16)600)

/* 1 hour in seconds */
#define ONE_HOUR_AS_SECONDS        ((u16)3600)

/* 10 hours in seconds */
#define TEN_HOURS_AS_SECONDS       ((u16)36000)

/***************Border**********************/
#define MIN_DISTANCE_LEFT          (4)
#define MAX_DISTANCE_RIGHT         (MIN_DISTANCE_LEFT + 7)

/*************** switches *******************/
#define SWITCH_UP                  (0)
#define SWITCH_DOWN                (1)
#define SWITCH_RIGHT               (2)
#define SWITCH_LEFT                (3)
#define SWITCH_OK                  (4)
#define SWITCH_DATE                (5)
#define SWITCH_CONFIG              (6)

/******************private functions*****************/
static void APP_UpdateSecondsCounter(u8 xPos, u8 pressedSwitch);
static void APP_PrintTimeAtInitialPosition(u8 colon);
static void APP_Task(void);

/******************* CallBack Function *************/
static void APP_UART_TC_Callback(void);

/************************** Global array ***********/
static u8 TxSwitchesStatus[7] ;
static u8 RxSwitchesStatus[7];

/**************** Global Variable ***********/
/* determines whether we're inside date mode or not */
static u8 Is_DateMode;

/* determines whether we're inside config mode or not */
static u8 Is_ConfigMode;

/* counts the total time passed in seconds */
static u32 TimeCounterSeconds;

/* the x-position of the cursor on the LCD */
static u8 xPos;

/* the total time passed in seconds converted to an array for the LCD */
static u8 timeArray[8];

/* scheduler object*/
SCHED_task_t const task_app = {
	.runnable = &APP_Task,
	.periodicTimeMs = APP_TASK_PERIODICITY_MS,
};

/************************** GPIO TX ***************************/
const GPIO_t UART1_TxPin = {
	.pin   = GPIO_PIN9_SELECT,
	.mode  = GPIO_PIN9_MODE_OUTPUT_AF_PUSH_PULL,
	.speed = GPIO_PIN9_SPEED_50MHZ,
	.port  = GPIO_PIN9_PORTA,
};

/**************************** GPIO RX **************************/
const GPIO_t UART1_RxPin = {
	.pin   = GPIO_PIN10_SELECT,
	.mode  = GPIO_PIN10_MODE_INPUT_FLOATING,
	.speed = GPIO_PIN10_SPEED_NONE,
	.port  = GPIO_PIN10_PORTA,
};

/************************ DMA Configuration ********************/
DMA_ChannelCfg_t const DMA_UART_TxConfiguation = {
	.peri = DMA_PERI1,
	.channel = DMA_CHANNEL4,

	.data_count = 7,
	.data_direction = DMA_DATA_DIRECTION_MEM2PERI,

	.peripheral_address = (void *)0x40013804, //address of UART_DR
	.memory_address = TxSwitchesStatus,

	.channel_priority = DMA_CHANNEL_PRIORITY_LOW,

	.circular_mode_control = DMA_CIRCULAR_MODE_CONTROL_OFF,
	.mem2mem_mode_control = DMA_MEM2MEM_MODE_CONTROL_OFF,

	.peripheral_inc_mode_control = DMA_PERIPHERAL_INC_MODE_CONTROL_OFF,
	.memory_inc_mode_control = DMA_MEMORY_INC_MODE_CONTROL_ON,

	.peripheral_size = DMA_PERIPHERAL_SIZE_32_BIT,
	.memory_size = DMA_MEMORY_SIZE_8_BIT,

	.transfer_error_int_control = DMA_TRANSFER_ERROR_INT_CONTROL_OFF,
	.half_transfer_int_control = DMA_HALF_TRANSFER_INT_CONTROL_OFF,
	.transfer_complete_int_control = DMA_TRANSFER_COMPLETE_INT_CONTROL_OFF,
};

DMA_ChannelCfg_t const DMA_UART_RxConfiguation = {
	.peri = DMA_PERI1,
	.channel = DMA_CHANNEL5,

	.data_count = 7,
	.data_direction = DMA_DATA_DIRECTION_PERI2MEM,

	.peripheral_address = (void *)0x40013804, //address of UART_DR
	.memory_address = RxSwitchesStatus,

	.channel_priority = DMA_CHANNEL_PRIORITY_LOW,

	.circular_mode_control = DMA_CIRCULAR_MODE_CONTROL_ON,
	.mem2mem_mode_control = DMA_MEM2MEM_MODE_CONTROL_OFF,

	.peripheral_inc_mode_control = DMA_PERIPHERAL_INC_MODE_CONTROL_OFF,
	.memory_inc_mode_control = DMA_MEMORY_INC_MODE_CONTROL_ON,

	.peripheral_size = DMA_PERIPHERAL_SIZE_32_BIT,
	.memory_size = DMA_MEMORY_SIZE_8_BIT,

	.transfer_error_int_control = DMA_TRANSFER_ERROR_INT_CONTROL_OFF,
	.half_transfer_int_control = DMA_HALF_TRANSFER_INT_CONTROL_OFF,
	.transfer_complete_int_control = DMA_TRANSFER_COMPLETE_INT_CONTROL_OFF,
};

/*********************** Main *********************/
void main(void)
{
	Is_DateMode = 0;
	Is_ConfigMode = 0;
	TimeCounterSeconds = 0;
	xPos = MIN_DISTANCE_LEFT;

	for (u8 i = 0; i < 7; i++)
	{
		RxSwitchesStatus[i] = SWITCH_UNPRESSED;
	}

	/* turn on HSE */
	RCC_SetClock(RCC_CR_HSE, ON);

	/* switch system clock to HSE */
	Select_SystemClock(RCC_CFGR_SW_HSE);

	/* HCLK = PPRECLK1, 2 = div1 */
	RCC_HPRE_SetPrescaler(RCC_CFGR_HPRE_div_1);
	RCC_PPRE1_SetPrescaler(RCC_CFGR_PPRE1_div_1);
	RCC_PPRE2_SetPrescaler(RCC_CFGR_PPRE2_div_1);

	/* enable clocks for ports A, B, C, USART 1, and DMA 1 */
	RCC_EnablePeripheral_APB2(RCC_APB2ENR_IOPAEN_PORTA);
	RCC_EnablePeripheral_APB2(RCC_APB2ENR_IOPBEN_PORTB);
	RCC_EnablePeripheral_APB2(RCC_APB2ENR_IOPCEN_PORTC);
	RCC_EnablePeripheral_APB2(RCC_APB2ENR_USART1EN);
	RCC_EnablePeripheral_AHB(RCC_AHBENR_DMA1EN);

	/* init Switch module */
	Switch_Init();

	/* init UART 1 pins */
	GPIO_InitPin(&UART1_TxPin);
	GPIO_InitPin(&UART1_RxPin);

	/* init UART 1,
	 * register TC complete callback and enable it's interrupt,
	 * and enable the DMA trigger on reception */
	UART_Init(UART_PERI_1, 9600);
	UART_ConfigTransmissionCompleteCallback(UART_PERI_1, &APP_UART_TC_Callback);
	UART_ControlTransmissionCompleteINT(UART_PERI_1, UART_TARNSMISSION_COMPLETE_CONTROL_INT_ON);
	UART_ControlDMAReceiverState(UART_PERI_1, UART_DMA_RECEIVER_STATE_ON);

	/* init DMA channels of UART 1 Tx and Rx */
	DMA_InitChannel(&DMA_UART_TxConfiguation);
	DMA_InitChannel(&DMA_UART_RxConfiguation);

	/* config DMA channel of UART 1 Rx */
	DMA_ControlChannel(&DMA_UART_RxConfiguation, DMA_CHANNEL_CONTROL_OFF);
	DMA_ConfigChannel(&DMA_UART_RxConfiguation);
	DMA_ControlChannel(&DMA_UART_RxConfiguation, DMA_CHANNEL_CONTROL_ON);

	/* enable IRQs of: UART 1, and DMA ch 4, 5 */
	NVIC_EnableIRQ(USART1_IRQNUMBER);
	NVIC_EnableIRQ(DMA1_Channel5_IRQHandler);
	NVIC_EnableIRQ(DMA1_Channel4_IRQHandler);

	/* turn off blinking and set cursor at initial left position/border */
	LCD_WriteCMD(LCD_CMD_DisplayOn_CursorOff_BlinkOff);
	LCD_WriteCMD(LCD_CMD_Set_DDRAM_Addr + MIN_DISTANCE_LEFT);

	/* init Scheduler */
	SCHED_Init();

	SCHED_Start();
}

/************************** Application Task ***********/

static void APP_UpdateSecondsCounter(u8 xPos, u8 pressedSwitch)
{
	u16 diff; /* difference value */

	if(xPos == MAX_DISTANCE_RIGHT) /* if at 1st digit of seconds */
	{
		/* 1 second */
		diff = 1;
	}
	else if(xPos == (MAX_DISTANCE_RIGHT - 1)) /* if at 2st digit of seconds */
	{
		/* 10 seconds */
		diff = 10;
	}
	else if (xPos == (MAX_DISTANCE_RIGHT - 3)) /* if at 1st digit of minutes */
	{
		/* 1 minute */
		diff = ONE_MINUTE_AS_SECONDS;
	}
	else if(xPos == (MAX_DISTANCE_RIGHT - 4)) /* if at 2nd digit of minutes */
	{
		/* 10 minutes */
		diff = TEN_MINUTES_AS_SECONDS;
	}
	else if (xPos == (MAX_DISTANCE_RIGHT - 6)) /* if at 1st digit of hours */
	{
		/* 1 hour */
		diff = ONE_HOUR_AS_SECONDS;
	}
	else if(xPos == (MAX_DISTANCE_RIGHT - 7)) /* if at 2nd digit of hours */
	{
		/* 10 hours */
		diff = TEN_HOURS_AS_SECONDS;
	}
	else
	{

	}

	if (pressedSwitch == SWITCH_UP)
	{
		TimeCounterSeconds += diff;

		/* this overflow happens when we add a big value */
		if (TimeCounterSeconds >= MAX_SECONDS)
		{
			TimeCounterSeconds = MAX_SECONDS;
		}
		else
		{

		}
	}
	else /* switch down is pressed */
	{
		TimeCounterSeconds -= diff;

		/* this underflow happens when we subtract a value that's more than original counter itself */
		if (TimeCounterSeconds >= MAX_SECONDS)
		{
			TimeCounterSeconds = 0;
		}
		else
		{

		}
	}

	/* this overflow happens when we add a big value,
	 * or we subtract a value that's more than original counter itself (underflow) */
	if (TimeCounterSeconds > MAX_SECONDS)
	{
		TimeCounterSeconds = 0;
	}
	else
	{

	}
}

static void APP_PrintTimeAtInitialPosition(u8 colon)
{
	/* prepare array */
	timeArray[0] = (TimeCounterSeconds / ONE_HOUR_AS_SECONDS) / 10 + '0';
	timeArray[1] = (TimeCounterSeconds / ONE_HOUR_AS_SECONDS) % 10 + '0';
	timeArray[2] = colon;
	timeArray[3] = ((TimeCounterSeconds % ONE_HOUR_AS_SECONDS) / ONE_MINUTE_AS_SECONDS) / 10 + '0';
	timeArray[4] = ((TimeCounterSeconds % ONE_HOUR_AS_SECONDS) / ONE_MINUTE_AS_SECONDS) % 10 + '0';
	timeArray[5] = colon;
	timeArray[6] = ((TimeCounterSeconds % ONE_HOUR_AS_SECONDS) % ONE_MINUTE_AS_SECONDS) / 10 + '0';
	timeArray[7] = ((TimeCounterSeconds % ONE_HOUR_AS_SECONDS) % ONE_MINUTE_AS_SECONDS) % 10 + '0';

	/* goto initial position*/
	LCD_WriteCMD(LCD_CMD_Set_DDRAM_Addr + MIN_DISTANCE_LEFT);

	/* write the array */
	LCD_WriteData(timeArray, 8);
}

static void APP_Task(void)
{
	static u8 MiliCounter = 0;     /* counts the amount of periodicity that's passed, used to increment time counter */
	static u8 colonDisplay = ':';  /* the colon to be displayed, either a colon or a blank space*/
	static u8 colonToggleCtr = 0;  /* timeout counter for when to toggle the colon */

	/* increment the millisec/periodicty counter (concerned with the time counter) */
	MiliCounter++;
	if (MiliCounter == (ONE_SECOND_AS_MILLISEC / APP_TASK_PERIODICITY_MS)) /* if one second has passed (as multiples of task periodicity) */
	{
		/* reset the counter back */
		MiliCounter = 0;

		if (Is_ConfigMode == 0) /* if we're not in config mode */
		{
			/* we can safely increment the time counter */
			TimeCounterSeconds++;
			/* this overflow happens when we add a big value */
			if (TimeCounterSeconds > MAX_SECONDS)
			{
				TimeCounterSeconds = 0;
			}
			else
			{

			}
		}
		else /* if we're in config mode, we shouldn't increment time counter */
		{

		}
	}
	else /* if one second didn't pass */
	{

	}

	/* increment the millisec/periodicty counter (concerned with the colon toggle) */
	colonToggleCtr++;
	if (colonToggleCtr == ((ONE_SECOND_AS_MILLISEC / 2) / APP_TASK_PERIODICITY_MS)) /* if 0.5 sec has passed (as multiples of task periodicity) */
	{
		colonToggleCtr = 0;

		if (Is_ConfigMode == 0) /* if we're not in config mode */
		{
			/* toggle the colon */
			if (colonDisplay == ':')
			{
				colonDisplay = ' ';
			}
			else /* colonDisplay == ' ' */
			{
				colonDisplay = ':';
			}
		}
		else
		{

		}
	}
	else
	{

	}

	/*****************************************transmitter*****************************/
	/* get the all the switches states and store them in the transmission array */
	TxSwitchesStatus[SWITCH_CONFIG] = Switch_GetReading(SWITCH_CONFIG);
	TxSwitchesStatus[SWITCH_UP]     = Switch_GetReading(SWITCH_UP);
	TxSwitchesStatus[SWITCH_DOWN]   = Switch_GetReading(SWITCH_DOWN);
	TxSwitchesStatus[SWITCH_LEFT]   = Switch_GetReading(SWITCH_LEFT);
	TxSwitchesStatus[SWITCH_RIGHT]  = Switch_GetReading(SWITCH_RIGHT);
	TxSwitchesStatus[SWITCH_DATE]   = Switch_GetReading(SWITCH_DATE);
	TxSwitchesStatus[SWITCH_OK]     = Switch_GetReading(SWITCH_OK);

	/* re-configure DMA Tx channel */
	DMA_ControlChannel(&DMA_UART_TxConfiguation, DMA_CHANNEL_CONTROL_OFF);
	DMA_ConfigChannel(&DMA_UART_TxConfiguation);
	DMA_ControlChannel(&DMA_UART_TxConfiguation, DMA_CHANNEL_CONTROL_ON);

	/* enable DMA trigger on Tx buffer empty event */
	UART_ControlDMATransmitterState(UART_PERI_1, UART_DMA_TRANSMITTER_STATE_ON);

	/*******************************************receive*****************************/
	if (RxSwitchesStatus[SWITCH_CONFIG] == SWITCH_PRESSED) /* if the config mode switch was pressed */
	{
		/* set the config mode flag */
		Is_ConfigMode = 1;

		/* turn on cursor */
		LCD_WriteCMD(LCD_CMD_DisplayOn_CursorOn_BlinkOff);
	}
	else /* if the config mode switch was NOT pressed */
	{

	}

	if (Is_ConfigMode == 1) /* if we're inside config mode */
	{
		if(RxSwitchesStatus[SWITCH_LEFT] == SWITCH_PRESSED) /* if the left switch was pressed */
		{
			if(xPos == MIN_DISTANCE_LEFT) /* if the xPos reached the left border */
			{
				/* set xPos to right border, circular increment */
				xPos = MAX_DISTANCE_RIGHT;
			}
			else /* if the xPos didn't reach the left border */
			{
				/* decrement the counter */
				xPos--;

				if((xPos == (MIN_DISTANCE_LEFT + 2)) || (xPos == (MIN_DISTANCE_LEFT + 5))) /* if we hit a colon ':' */
				{
					/* decrement the counter to move past the colon */
					xPos--;
				}
				else /* if we didn't hit a colon ':' */
				{

				}
			}
		}
		else /* if the left switch was NOT pressed */
		{

		}

		if(RxSwitchesStatus[SWITCH_RIGHT] == SWITCH_PRESSED) /* if the right switch was pressed */
		{
			if(xPos == MAX_DISTANCE_RIGHT) /* if the xPos reached the right border */
			{
				/* set xPos to left border, circular increment */
				xPos =  MIN_DISTANCE_LEFT;
			}
			else /* if the xPos didn't reach the right border */
			{
				/* increment the counter */
				xPos++;

				if((xPos == (MIN_DISTANCE_LEFT + 2)) || (xPos == (MIN_DISTANCE_LEFT + 5))) /* if we hit a colon ':' */
				{
					/* increment the counter to move past the colon */
					xPos++;
				}
				else /* if we didn't hit a colon ':' */
				{

				}
			}
		}
		else /* if the right switch was NOT pressed */
		{

		}

		if(RxSwitchesStatus[SWITCH_UP] == SWITCH_PRESSED) /* if up switch is pressed */
		{
			/* update the seconds counter */
			APP_UpdateSecondsCounter(xPos, SWITCH_UP);
		}
		else /* if up switch is NOT pressed */
		{

		}

		/*************************************down pressed ***************************************///
		if(RxSwitchesStatus[SWITCH_DOWN] == SWITCH_PRESSED) /* if down switch is pressed */
		{
			/* update the seconds counter */
			APP_UpdateSecondsCounter(xPos, SWITCH_DOWN);
		}
		else /* if down switch is NOT pressed */
		{

		}

		/* print the time after updating the time counter and the xPos */
		APP_PrintTimeAtInitialPosition(colonDisplay);

		/* put cursor at config location */
		LCD_WriteCMD(LCD_CMD_Set_DDRAM_Addr + xPos);
	}
	else  /* if we're NOT inside config mode */
	{
		if(RxSwitchesStatus[SWITCH_DATE] == SWITCH_PRESSED)
		{
			if (Is_DateMode == 0) /* if 1st time entering data mode */
			{
				Is_DateMode = 1;

				/* clear screen */
				LCD_WriteCMD(LCD_CMD_Clear_Display);
			}
			else /* if NOT 1st time entering data mode (we previously entered date mode) */
			{

			}

			/* goto initial position*/
			LCD_WriteCMD(LCD_CMD_Set_DDRAM_Addr + MIN_DISTANCE_LEFT);

			/* print date */
			LCD_WriteData("14 MAY 2020", 11); /* the date when this project was made :D */
		}
		else if ((RxSwitchesStatus[SWITCH_DATE] == SWITCH_UNPRESSED) && (Is_DateMode == 1))
		{ /* if the switch was released and we previously was inside Date mode */
			Is_DateMode = 0;

			/* clear screen */
			LCD_WriteCMD(LCD_CMD_Clear_Display);

			/* print time back again */
			APP_PrintTimeAtInitialPosition(colonDisplay);
		}
		else /* normal time (nothing is pressed) */
		{
			APP_PrintTimeAtInitialPosition(colonDisplay);
		}
	}

	if (RxSwitchesStatus[SWITCH_OK] == SWITCH_PRESSED) /* if the OK switch was pressed */
	{
		/* reset the config mode flag */
		Is_ConfigMode = 0;

		/* turn off cursor */
		LCD_WriteCMD(LCD_CMD_DisplayOn_CursorOff_BlinkOff);
	}
	else /* if the OK switch was NOT pressed */
	{

	}
}

/************************* UART TC Callback ***********************/
static void APP_UART_TC_Callback(void)
{
	/* after transmission is complete, turn off DMA trigger on Tx bugger empty event */
	UART_ControlDMATransmitterState(UART_PERI_1, UART_DMA_TRANSMITTER_STATE_OFF);
}

