#include "STD_TYPES.h"
#include "sched_interface.h"
#include "sched_config.h"
#include "SYSTICK.h"

typedef struct
{
	SCHED_systask_info_t const *task;
	u32 remainToExec;
	u32 periodTicks;
	u32 state;
} SCHED_systask_t;

/* array defined in the config.c file */
extern const SCHED_systask_info_t tasks_config[SCHED_MAX_TASKS];

/* array of tasks */
static SCHED_systask_t systask[SCHED_MAX_TASKS];

/* raised when tick time is over */
static volatile u8 os_flag = 0;

static void SCHED_SetOSFlag(void);

static void SCHED_Loop(void);

void SCHED_Init(void)
{
	u8 i;
	for (i = 0; i < SCHED_MAX_TASKS; i++)
	{
		/* refer to the task runnable */
		systask[i].task = &tasks_config[i];

		/* first delay hybd2 emta */
		systask[i].remainToExec = tasks_config[i].delayMs / SCHED_TICK_TIME;

		/* deh hygy kol 2d eh :D */
		systask[i].periodTicks = tasks_config[i].apptask->periodicTimeMs
				/ SCHED_TICK_TIME;

		/* malnash d3wa beh nw */
		systask[i].state = 0;
	}
}

void SCHED_Start(void)
{
	SYSTICK_Init();
	SYSTICK_SetTimers(SCHED_TICK_TIME);
	SYSTICK_SetCallBack(&SCHED_SetOSFlag);
	SYSTICK_Start();

	while (1)
	{
		SCHED_Loop();
	}
}

static void SCHED_SetOSFlag(void)
{
	os_flag = 1;
}

static void SCHED_Loop(void)
{
	u8 i;

	if (os_flag == 1) /* if OS flag was raised */
	{
		os_flag = 0; /* reset the flag */

		for (i = 0; i < SCHED_MAX_TASKS; i++) /* foreach task */
		{
			if (systask[i].remainToExec == 0) /* if task has timed out */
			{
				systask[i].task->apptask->runnable(); /* call its runnable */
				systask[i].remainToExec = systask[i].periodTicks; /* reset its remaining ticks */
			}

			/* decrement the remaining ticks counter */
			systask[i].remainToExec--;
		}
	}
	else /* if OS flag was not raised */
	{

	}

}
