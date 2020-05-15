#ifndef UART_H
#define UART_H

#define UART_PERI_1                               0
#define UART_PERI_2                               1
#define UART_PERI_3                               2

#define UART_STATE_CONTROL_ON                           0x00002000
#define UART_STATE_CONTROL_OFF                          0x00000000

#define UART_DATA_LENGTH_9BIT                           0x00001000
#define UART_DATA_LENGTH_8BIT                           0x00000000

#define UART_PARITY_STATE_ON                            0x00000400
#define UART_PARITY_STATE_OFF                           0x00000000

#define UART_PARITY_SELECT_EVEN                         0x00000000
#define UART_PARITY_SELECT_ODD                          0x00000200

#define UART_PARITY_CONTROL_INT_ON                      0x00000100
#define UART_PARITY_CONTROL_INT_OFF                     0x00000000

#define UART_TX_BUFFER_EMPTY_CONTROL_INT_ON             0x00000080
#define UART_TX_BUFFER_EMPTY_CONTROL_INT_OFF            0x00000000

#define UART_TARNSMISSION_COMPLETE_CONTROL_INT_ON       0x00000040
#define UART_TARNSMISSION_COMPLETE_CONTROL_INT_OFF      0x00000000

#define UART_RX_BUFFER_FULL_CONTROL_INT_ON              0x00000020
#define UART_RX_BUFFER_FULL_CONTROL_INT_OFF             0x00000000

#define UART_TANSMITTER_STATE_ON                        0x00000008
#define UART_TANSMITTER_STATE_OFF                       0x00000000

#define UART_RECEIVER_STATE_ON                          0x00000004
#define UART_RECEIVER_STATE_OFF                         0x00000000

#define UART_LIN_STATE_CONTROL_ON                       0x00004000
#define UART_LIN_STATE_CONTROL_OFF                      0x00000000

#define UART_STOP_BITS_1                                0x00000000
#define UART_STOP_BITS_2                                0x00002000

#define UART_LIN_BREAK_DETECTION_CONTROL_INT_ON         0x00000040
#define UART_LIN_BREAK_DETECTION_CONTROL_INT_OFF        0x00000000

#define UART_LIN_BREAK_DETECTION_LENGTH_10              0x00000000
#define UART_LIN_BREAK_DETECTION_LENGTH_11              0x00000020

#define UART_DMA_TRANSMITTER_STATE_ON                   0x00000080
#define UART_DMA_TRANSMITTER_STATE_OFF                  0x00000000

#define UART_DMA_RECEIVER_STATE_ON                      0x00000040
#define UART_DMA_RECEIVER_STATE_OFF                     0x00000000

#define UART_DMA_ERROR_CONTROL_INT_ON                   0x00000001
#define UART_DMA_ERROR_CONTROL_INT_OFF                  0x00000000

typedef void (*UART_TxBufferEmptyCallback_t)(void);
typedef void (*UART_RxBufferFullCallback_t)(u16);
typedef void (*UART_ParityErrorCallback_t)(void);
typedef void (*UART_TranssmisonCompleteCallback_t)(void);
typedef void (*UART_LINBreakCallback_t)(void);
typedef void (*UART_DMAErrorCallback_t)(void);
typedef void (*UART_ErrorFrameCallback_t)(void);
typedef void (*UART_NoiseErrorCallback_t)(void);
typedef void (*UART_OverRunErrorCallback_t)(void);

void UART_Init(u8 peri, u32 baudRate);

u8 UART_GetIsLIN_BreakDetection(u8 peri);
u8 UART_GetIsTxBufferEmpty(u8 peri);
u8 UART_GetIsTansmitComplete(u8 peri);
u8 UART_GetIsRxBufferFull(u8 peri);
u8 UART_GetIsOverRunError(u8 peri);
u8 UART_GetINoiseError(u8 peri);
u8 UART_GetIsFrameError(u8 peri);
u8 UART_GetIsParityError(u8 peri);
u16 UART_GetData(u8 peri);

void UART_ConfigData(u8 peri, u16 data);
void UART_ConfigBaudRate(u8 peri, u16 baud_rate);
void UART_ConfigParityType(u8 peri, u32 parity_select);
void UART_ConfigStopBits(u8 peri, u32 stop_bits);
void UART_ConfigLINBreakDetectionLength(u8 peri, u32 lin_break_detection_length);
void UART_ConfigTxBufferEmptyCallback(u8 peri, UART_TxBufferEmptyCallback_t ptr);
void UART_ConfigRxBufferFullCallback(u8 peri, UART_RxBufferFullCallback_t ptr);
void UART_ConfigParityErrorCallback(u8 peri, UART_ParityErrorCallback_t ptr);
void UART_ConfigTransmissionCompleteCallback(u8 peri, UART_TranssmisonCompleteCallback_t ptr);
void UART_ConfigLINBreakCallback(u8 peri, UART_LINBreakCallback_t ptr);
void UART_ConfigNoiseErrorCallBack(u8 peri, UART_NoiseErrorCallback_t ptr);
void UART_ConfigOverRunErrorCallBack(u8 peri, UART_OverRunErrorCallback_t ptr);
void UART_ConfigFrameErrorCallBack(u8 peri, UART_ErrorFrameCallback_t ptr);

void UART_ControlState(u8 peri, u32 state_control);
void UART_ControlDataLength(u8 peri, u32 data_length);
void UART_ControlParityState(u8 peri, u32 parity_state);
void UART_ControlParityErrorINT(u8 peri, u32 parity_control_INT);
void UART_ControlTxBufferEmptyINT(u8 peri, u32 tx_buffer_empty_control_INT);
void UART_ControlTransmissionCompleteINT(u8 peri, u32 transmission_complete_control_INT);
void UART_ControlRxBufferFullINT(u8 peri, u32 rx_buffer_full_control_INT);
void UART_ControlLINBreakDetectionINT(u8 peri, u32 lin_break_detection_control_INT);
void UART_ControlDMAErrorINT(u8 peri, u32 dma_error_control_INT);
void UART_ControlTransmitterState(u8 peri, u32 transmit_state);
void UART_ControlReceiverState(u8 peri, u32 receive_state);
void UART_ControlLINState(u8 peri, u32 lin_state_control);
void UART_ControlDMATransmitterState(u8 peri, u32 dma_transmitter_state);
void UART_ControlDMAReceiverState(u8 peri, u32 dma_receivier_state);

void UART_SetBreakCharacter(u8 peri);

#endif /* UART_H */

