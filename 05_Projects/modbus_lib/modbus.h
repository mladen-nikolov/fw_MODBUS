/* *****************************************************************************
 * File:   modbus.h
 * Author: XX
 *
 * Created on 2023 03 10
 * 
 * Description: ...
 * 
 **************************************************************************** */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */


/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include <stdint.h>
    
/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define MODBUS_TIMEOUT_BYTE_MS                 2,5           /* max timeout between bytes to reset the message in receive communication loop also on timeout triggers evaluation of received till now message */
#define MODBUS_TIMEOUT_MESSAGE_MS              2,5           /* timeout from the last byte received before say - detected end of the message */
#define MODBUS_TIMEOUT_MESS_MESS_MS            1,5           /* timeout from the end of previous message to the start of the next message to detect start of message */


/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macro
 **************************************************************************** */

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */ 

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */
void modbus_init (void);
void modbus_holding_register_address_set(uint16_t register_number, uint32_t data_address);
void modbus_uart_read_fast_loop_process (void);
void modbus_main_loop_process (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */


