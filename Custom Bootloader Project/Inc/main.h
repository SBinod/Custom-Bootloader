/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN EM */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000

/*Bootloader commands*/

/*#define      <command_name>          <command_code>*/
#define         BL_GET_VER                  0x51
#define         BL_GET_HELP                 0x52
#define         BL_GET_CID                  0x53
#define         BL_RDP_STATUS               0x54
#define         BL_GO_TO_ADDR               0x55
#define         BL_FLASH_ERASE              0x56
#define         BL_MEM_WRITE                0x57
#define         BL_ENDIS_RW_PROTECT         0x58
#define         BL_MEM_READ                 0x59
#define         BL_READ_SECTOR_STATUS       0x5A

#define         BL_NACK                     0x7F
#define         BL_ACK                      0xA5

#define         VERIFY_CRC_SUCCESS          0
#define         VERIFY_CRC_FAIL             1

#define         BL_VERSION                  0x10

#define         ADDR_VALID                  0x00
#define         ADDR_INVALID                0x01

/*
 * Start and End addresses of different memories of STM32F446xx MCU 
 * Change this according to the MCU being used
 */
#define         SRAM1_SIZE                  112*1024                    /*STM32F446RE has 112KB of SRAM1*/
#define         SRAM1_END                   (SRAM1_BASE + SRAM1_SIZE)
#define         SRAM2_SIZE                  16*1024                     /*STM32F446RE has 016KB of SRAM2*/
#define         SRAM2_END                   (SRAM2_BASE + SRAM2_SIZE)
#define         FLASH_SIZE                  512*1024                    /*STM32F446RE has 512KB of SRAM2*/
#define         BKPSRAM_SIZE                4*1024                      /*STM32F446RE has 004KB of SRAM2*/
#define         BKPSRAM_END                 (BKPSRAM_BASE + BKPSRAM_SIZE)

#define         LD2_Pin                     GPIO_PIN_5
#define         LD2_GPIO_Port               GPIOA

#define         INVALID_SECTOR              0x04
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_rdpstatus_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_gotoaddr_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_flasherase_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_memwrite_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_endisrwprotect_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_memread_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_readsectorstatus_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_otpread_cmd(uint8_t* bl_rx_buffer);

void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc(uint8_t* pData, uint32_t len, uint32_t crc_host);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len);
uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);
uint16_t read_OB_rw_protection_status(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
