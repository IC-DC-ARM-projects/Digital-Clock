#ifndef DMA_H_
#define DMA_H_

/* DMA peripheral selectors */
/**
 * @brief DMA callback type, used when registering a channel event callback
 *
 */
typedef void (*DMA_Callback_t)(void);

typedef struct
{
	/**
	 * DMA peripheral select:\n
	 * DMA_PERI1,\n
	 * DMA_PERI2
	 */
	u8 peri;

	/**
	 * DMA channel select:\n
	 * DMA_CHANNEL1,\n
	 * DMA_CHANNEL2,\n
	 * DMA_CHANNEL3,\n
	 * DMA_CHANNEL4,\n
	 * DMA_CHANNEL5,\n
	 * DMA_CHANNEL6,\n
	 * DMA_CHANNEL7
	 */
	u8 channel;


	/**
	 * DMA channel transfer data unit count/length: [0 -> 65535]
	 */
	u16 data_count;

	/**
	 * DMA channel transfer direction:\n
	 * DMA_DATA_DIRECTION_PERI2MEM,\n
	 * DMA_DATA_DIRECTION_MEM2PERI
	 */
	u32 data_direction;


	/**
	 * DMA channel peripheral address
	 */
	void* peripheral_address;

	/**
	 * DMA channel memory address
	 */
	void* memory_address;


	/**
	 * DMA channel priority:\n
	 * DMA_CHANNEL_PRIORITY_LOW,\n
	 * DMA_CHANNEL_PRIORITY_MEDIUM,\n
	 * DMA_CHANNEL_PRIORITY_HIGH,\n
	 * DMA_CHANNEL_PRIORITY_VERY_HIGH
	 */
	u32 channel_priority;


	/**
	 * DMA channel circular mode enable/disable:\n
	 * DMA_CIRCULAR_MODE_CONTROL_OFF,\n
	 * DMA_CIRCULAR_MODE_CONTROL_ON
	 */
	u32 circular_mode_control;

	/**
	 * DMA channel memory-to-memory mode enable/disable:\n
	 * DMA_MEM2MEM_MODE_CONTROL_OFF,\n
	 * DMA_MEM2MEM_MODE_CONTROL_ON
	 */
	u32 mem2mem_mode_control;


	/**
	 * DMA channel peripheral address increment mode enable/disable:\n
	 * DMA_PERIPHERAL_INC_MODE_CONTROL_OFF,\n
	 * DMA_PERIPHERAL_INC_MODE_CONTROL_ON
	 */
	u32 peripheral_inc_mode_control;

	/**
	 * DMA channel memory address increment mode enable/disable:\n
	 * DMA_MEMORY_INC_MODE_CONTROL_OFF,\n
	 * DMA_MEMORY_INC_MODE_CONTROL_ON
	 */
	u32 memory_inc_mode_control;


	/**
	 * DMA channel peripheral data unit size:\n
	 * DMA_PERIPHERAL_SIZE_8_BIT,\n
	 * DMA_PERIPHERAL_SIZE_16_BIT,\n
	 * DMA_PERIPHERAL_SIZE_32_BIT
	 */
	u32 peripheral_size;

	/**
	 * DMA channel memory data unit size:\n
	 * DMA_MEMORY_SIZE_8_BIT,\n
	 * DMA_MEMORY_SIZE_16_BIT,\n
	 * DMA_MEMORY_SIZE_32_BIT
	 */
	u32 memory_size;


	/**
	 * DMA channel transfer-error event interrupt enable/disable:\n
	 * DMA_TRANSFER_ERROR_INT_CONTROL_OFF,\n
	 * DMA_TRANSFER_ERROR_INT_CONTROL_ON
	 */
	u32 transfer_error_int_control;

	/**
	 * DMA channel half-transfer event interrupt enable/disable:\n
	 * DMA_HALF_TRANSFER_INT_CONTROL_OFF,\n
	 * DMA_HALF_TRANSFER_INT_CONTROL_ON
	 */
	u32 half_transfer_int_control;

	/**
	 * DMA channel transfer-complete event interrupt enable/disable:\n
	 * DMA_TRANSFER_COMPLETE_INT_CONTROL_OFF,\n
	 * DMA_TRANSFER_COMPLETE_INT_CONTROL_ON
	 */
	u32 transfer_complete_int_control;


	/**
	 * DMA channel transfer-error event callback
	 */
	DMA_Callback_t transfer_error_callback;

	/**
	 * DMA channel half-transfer event callback
	 */
	DMA_Callback_t half_transfer_callback;

	/**
	 * DMA channel transfer-complete event callback
	 */
	DMA_Callback_t transfer_complete_callback;
} DMA_ChannelCfg_t;

/**
 * @brief Selector for DMA peripheral 1
 *
 */
#define DMA_PERI1                                                    0

/**
 * @brief Selector for DMA peripheral 2
 *
 */
#define DMA_PERI2                                                    1


/* DMA channel selectors */
/**
 * @brief Selector for DMA channel 1
 *
 */
#define DMA_CHANNEL1                                                 0

/**
 * @brief Selector for DMA channel 2
 *
 */
#define DMA_CHANNEL2                                                 1

/**
 * @brief Selector for DMA channel 3
 *
 */
#define DMA_CHANNEL3                                                 2

/**
 * @brief Selector for DMA channel 4
 *
 */
#define DMA_CHANNEL4                                                 3

/**
 * @brief Selector for DMA channel 5
 *
 */
#define DMA_CHANNEL5                                                 4

/**
 * @brief Selector for DMA channel 6
 *
 */
#define DMA_CHANNEL6                                                 5

/**
 * @brief Selector for DMA channel 7
 *
 */
#define DMA_CHANNEL7                                                 6


/* DMA channel priority selectors */
/**
 * @brief Selector for channel priority: Low
 *
 */
#define DMA_CHANNEL_PRIORITY_LOW                                     0x00000000

/**
 * @brief Selector for channel priority: Medium
 *
 */
#define DMA_CHANNEL_PRIORITY_MEDIUM                                  0x00001000

/**
 * @brief Selector for channel priority: High
 *
 */
#define DMA_CHANNEL_PRIORITY_HIGH                                    0x00002000

/**
 * @brief Selector for channel priority: Very High
 *
 */
#define DMA_CHANNEL_PRIORITY_VERY_HIGH                               0x00003000


/* DMA memory and peripheral data unit size selectors */
/**
 * @brief Selector for the size of the memory data unit: 8-bit (1 byte)
 *
 */
#define DMA_MEMORY_SIZE_8_BIT                                        0x00000000

/**
 * @brief Selector for the size of the memory data unit: 16-bit (2 bytes)
 *
 */
#define DMA_MEMORY_SIZE_16_BIT                                       0x00000400

/**
 * @brief Selector for the size of the memory data unit: 32-bit (4 bytes)
 *
 */
#define DMA_MEMORY_SIZE_32_BIT                                       0x00000800

/**
 * @brief Selector for the size of the peripheral data unit: 8-bit (1 byte)
 *
 */
#define DMA_PERIPHERAL_SIZE_8_BIT                                    0x00000000

/**
 * @brief Selector for the size of the peripheral data unit: 16-bit (2 byte)
 *
 */
#define DMA_PERIPHERAL_SIZE_16_BIT                                   0x00000100

/**
 * @brief Selector for the size of the peripheral data unit: 32-bit (4 byte)
 *
 */
#define DMA_PERIPHERAL_SIZE_32_BIT                                   0x00000200


/* DMA transfer direction */
/**
 * @brief Select the channel transfer direction as: peripheral -> memory
 *
 */
#define DMA_DATA_DIRECTION_PERI2MEM                                  0x00000000

/**
 * @brief Select the channel transfer direction as: memory -> peripheral
 *
 */
#define DMA_DATA_DIRECTION_MEM2PERI                                  0x00000010


/* DMA various controls on/off */
/**
 * @brief Control selector for the memory-to-memory mode: off
 *
 */
#define DMA_MEM2MEM_MODE_CONTROL_OFF                                 0x00000000

/**
 * @brief Control selector for the memory-to-memory mode: on
 *
 */
#define DMA_MEM2MEM_MODE_CONTROL_ON                                  0x00004000

/**
 * @brief Control selector for the memory increment mode: off
 *
 */
#define DMA_MEMORY_INC_MODE_CONTROL_OFF                              0x00000000

/**
 * @brief Control selector for the memory increment mode: on
 *
 */
#define DMA_MEMORY_INC_MODE_CONTROL_ON                               0x00000080

/**
 * @brief Control selector for the peripheral increment mode: off
 *
 */
#define DMA_PERIPHERAL_INC_MODE_CONTROL_OFF                          0x00000000

/**
 * @brief Control selector for the peripheral increment mode: on
 *
 */
#define DMA_PERIPHERAL_INC_MODE_CONTROL_ON                           0x00000040

/**
 * @brief Control selector for the circular mode: off
 *
 */
#define DMA_CIRCULAR_MODE_CONTROL_OFF                                0x00000000

/**
 * @brief Control selector for the circular mode: on
 *
 */
#define DMA_CIRCULAR_MODE_CONTROL_ON                                 0x00000020

/**
 * @brief Control selector for the transfer-error event interrupt: off
 *
 */
#define DMA_TRANSFER_ERROR_INT_CONTROL_OFF                           0x00000000

/**
 * @brief Control selector for the transfer-error event interrupt: on
 *
 */
#define DMA_TRANSFER_ERROR_INT_CONTROL_ON                            0x00000008

/**
 * @brief Control selector for the half-transfer event interrupt: off
 *
 */
#define DMA_HALF_TRANSFER_INT_CONTROL_OFF                            0x00000000

/**
 * @brief Control selector for the half-transfer event interrupt: on
 *
 */
#define DMA_HALF_TRANSFER_INT_CONTROL_ON                             0x00000004

/**
 * @brief Control selector for the transfer-complete event interrupt: off
 *
 */
#define DMA_TRANSFER_COMPLETE_INT_CONTROL_OFF                        0x00000000

/**
 * @brief Control selector for the transfer-complete event interrupt: on
 *
 */
#define DMA_TRANSFER_COMPLETE_INT_CONTROL_ON                         0x00000002

/**
 * @brief Control selector for the channel state: off
 *
 */
#define DMA_CHANNEL_CONTROL_OFF                                      0x00000000

/**
 * @brief Control selector for the channel state: on
 *
 */
#define DMA_CHANNEL_CONTROL_ON                                       0x00000001


/* DMA callback types */
/**
 * @brief Selector for the callback trigger event type: transfer-error
 *
 */
#define DMA_CALLBACK_TYPE_TRANSFER_ERROR                             0

/**
 * @brief Selector for the callback trigger event type: transfer-complete
 *
 */
#define DMA_CALLBACK_TYPE_TRANSFER_COMPLETE                          1

/**
 * @brief Selector for the callback trigger event type: half-transfer
 *
 */
#define DMA_CALLBACK_TYPE_HALF_TRANSFER                              2


/* initialization API */
/**
 * @brief Initialize a DMA channel and reset its configurations
 *
 * @param cfg: Reference to an initialized DMA_ChannelCfg_t object that holds all the channel configurations
 */
void DMA_InitChannel(const DMA_ChannelCfg_t* cfg);


/* getter APIs */
/**
 * @brief Get the state of the transfer-error channel flag
 *
 * @param cfg: Reference to an initialized DMA_ChannelCfg_t object that holds all the channel configurations
 * @return Boolean value: 1 => a transfer-error has occurred, 0 => no transfer-error occurred
 */
u8 DMA_GetIsErrorTransfer(const DMA_ChannelCfg_t* cfg);

/**
 * @brief Get the state of the half-transfer channel flag
 *
 * @param cfg: Reference to an initialized DMA_ChannelCfg_t object that holds all the channel configurations
 * @return Boolean value: 1 => half-transfer has finished, 0 => half-transfer didn't finish
 */
u8 DMA_GetIsHalfTransfer(const DMA_ChannelCfg_t* cfg);

/**
 * @brief Get the state of the transfer-complete channel flag
 *
 * @param cfg: Reference to an initialized DMA_ChannelCfg_t object that holds all the channel configurations
 * @return Boolean value: 1 => transfer has finished completely, 0 => transfer didn't finish completely
 */
u8 DMA_GetIsTransferComplete(const DMA_ChannelCfg_t* cfg);

/**
 * @brief Get the state of the global channel flag
 *
 * @param cfg: Reference to an initialized DMA_ChannelCfg_t object that holds all the channel configurations
 * @return Boolean value: 1 => an event has happened on the channel:\n
 *                             * Transfer error event (use DMA_GetIsErrorTransfer),\n
 *                             * Half transfer complete event (use DMA_GetIsHalfTransfer),\n
 *                             * Transfer complete event (use DMA_GetIsTransferComplete)
 *                        0 => no event happened on the channel
 */
u8 DMA_GetIsGlobalInterrupt(const DMA_ChannelCfg_t* cfg);


/* configuration API */
void DMA_ConfigChannel(const DMA_ChannelCfg_t* cfg);


/* control API */
/**
 * @brief Enable/Disable a channel
 *
 * @param cfg: Reference to an initialized DMA_ChannelCfg_t object that holds all the channel configurations
 * @param channel_control: The state of the channel:\n
 *                         DMA_CHANNEL_CONTROL_OFF\n
 *                         DMA_CHANNEL_CONTROL_ON
 */
void DMA_ControlChannel(const DMA_ChannelCfg_t* cfg, u32 channel_control);

#endif /* DMA_H_ */
