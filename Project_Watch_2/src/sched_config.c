#include "STD_TYPES.h"
#include "sched_interface.h"
#include "sched_config.h"

/* tasks objects defined in each corresponding module */
extern SCHED_task_t const task_switch,
                          task_lcd,
                          task_app;

/* each object is defined by:
 * task_t reference,
 * initial delay (ms)  */
const SCHED_systask_info_t tasks_config[SCHED_MAX_TASKS] = {
	{
		.apptask = &task_switch,
		.delayMs = 0
	},
	{
		.apptask = &task_lcd,
		.delayMs = 2
	},
	{
		.apptask = &task_app,
		.delayMs = 4
	},
};

