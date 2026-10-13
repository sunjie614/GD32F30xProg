#ifndef HARDWARE_TEST_CAN_H
#define HARDWARE_TEST_CAN_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    uint32_t tx_sfid, tx_efid;
    uint8_t tx_ff, tx_ft, tx_dlen, tx_data[8];
} can_transmit_message_struct;
typedef struct
{
    uint32_t rx_sfid, rx_efid;
    uint8_t rx_ff, rx_ft, rx_dlen, rx_data[8];
} can_receive_message_struct;
#define CAN0 0U
#define CAN_FF_STANDARD 0U
#define CAN_FF_EXTENDED 1U
#define CAN_FT_DATA 0U
#define CAN_FT_REMOTE 1U
#define CAN_NOMAILBOX 4U
uint8_t can_message_transmit(uint32_t can, can_transmit_message_struct* message);
bool CAN_Buffer_Get(can_receive_message_struct* message);
#endif
