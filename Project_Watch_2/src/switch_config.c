#include "STD_TYPES.h"
#include "switch_interface.h"
#include "GPIO.h"
#include "switch_config.h"

/* each object is defined by:
 * GPIO_t: object of GPIO,
 * activeState: SWITCH_ACTIVE_LOW, SWITCH_ACTIVE_HIGH */
Switch_cfg_t const sw_cfg[SWITCH_COUNT] = {

   /******** switch up *********/
   {
      .switchIO = {
			.pin = GPIO_PIN7_SELECT ,
			.port = GPIO_PIN_ALL_PORTA ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },

   /******** switch down *********/
   {
      .switchIO = {
			.pin = GPIO_PIN8_SELECT ,
			.port = GPIO_PIN_ALL_PORTA ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },

   /******** switch right *********/
   {
      .switchIO = {
			.pin = GPIO_PIN11_SELECT ,
			.port = GPIO_PIN_ALL_PORTA ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },

   /******** switch left *********/
   {
      .switchIO = {
			.pin = GPIO_PIN12_SELECT ,
			.port = GPIO_PIN_ALL_PORTA ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },

   /******** switch ok *********/
   {
      .switchIO = {
			.pin = GPIO_PIN9_SELECT ,
			.port = GPIO_PIN_ALL_PORTB ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },

   /******** switch date *********/
   {
      .switchIO = {
			.pin = GPIO_PIN8_SELECT ,
			.port = GPIO_PIN_ALL_PORTB ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },

   /******** switch config *********/
   {
      .switchIO = {
			.pin = GPIO_PIN7_SELECT ,
			.port = GPIO_PIN_ALL_PORTB ,
			.mode = GPIO_PIN_ALL_MODE_INPUT_PULL_UP_DOWN,
			.speed = GPIO_PIN_ALL_SPEED_NONE,
      },
	  .activeState = SWITCH_ACTIVE_LOW,
   },
};
