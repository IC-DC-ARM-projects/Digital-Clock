#include "STD_TYPES.h"
#include "switch_interface.h"
#include "GPIO.h"
#include "switch_config.h"
#include "sched_interface.h"

#define SWITCH_STATE_THRESHOLD 5

typedef struct
{
	u8 switchReading; /* will only be updated when the switch state becomes stable */
	u64 previousReading;
	u8 counter;
} Switch_t;

/* array of switch objects, each has: {switchReading, previousReading, counter} */
static Switch_t sw_obj[SWITCH_COUNT];

static void Switch_Task(void);

/* scheduler object, defined by:
 * runnable function,
 * period (ms) */
SCHED_task_t const task_switch = {
	.runnable = &Switch_Task,
	.periodicTimeMs = 6,
};

/* object from config.c */
extern Switch_cfg_t sw_cfg[SWITCH_COUNT];

void Switch_Init(void)
{
	/* init pins directions, modes, speeds */
	u8 i;

	for (i = 0; i < SWITCH_COUNT; i++)
	{
		GPIO_InitPin(&sw_cfg[i].switchIO);

		/* enable pull up/down depending on activeState */
		switch (sw_cfg[i].activeState)
		{
		default:
		case SWITCH_ACTIVE_HIGH:
			GPIO_WritePin(&sw_cfg[i].switchIO, GPIO_PIN_ALL_VALUE_LOW);
			sw_obj[i].switchReading = SWITCH_UNPRESSED;
		break;

		case SWITCH_ACTIVE_LOW:
			GPIO_WritePin(&sw_cfg[i].switchIO, GPIO_PIN_ALL_VALUE_HIGH);
			sw_obj[i].switchReading = SWITCH_PRESSED;
		break;
		}
	}
}

u8 Switch_GetReading(u8 switchNum)
{
	switch (sw_cfg[switchNum].activeState)
	{
	default:
	case SWITCH_ACTIVE_HIGH:
		return sw_obj[switchNum].switchReading;
	break;

	case SWITCH_ACTIVE_LOW:
		return !sw_obj[switchNum].switchReading;
	break;
	}
}


static void Switch_Task(void)
{
	u64 currentReading;

	for (u8 i = 0; i < SWITCH_COUNT; i++)
	{
		currentReading = GPIO_ReadPin(&sw_cfg[i].switchIO);

		if (currentReading == sw_obj[i].previousReading)
		{
			sw_obj[i].counter++;
		}
		else
		{
			sw_obj[i].counter = 0;
		}

		sw_obj[i].previousReading = currentReading;

		if (sw_obj[i].counter == SWITCH_STATE_THRESHOLD) /* if switch reading counter was stable for a long time */
		{
			sw_obj[i].previousReading = 0;
			sw_obj[i].counter = 0;

			switch (currentReading)
			{
			case GPIO_PIN_ALL_VALUE_LOW: /* if GND */
				sw_obj[i].switchReading = SWITCH_UNPRESSED;
			break;

			default: /* if VCC */
				sw_obj[i].switchReading = SWITCH_PRESSED;
			break;
			}
		}
		else /* if switch reading counter was not stable enough for a long time */
		{

		}
	}
}
