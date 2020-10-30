/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdint.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BL_DEBUG_MSG_EN
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_STATUS} ;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void printmsg(char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char somedata[]= "Hello from Bootloader\r\n";
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
	/*Check whether user button is pressed or not and take appropriate action*/
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		printmsg("BL_DEBUG_MSG: Button is pressed... Going to BL Mode\n\r");
		bootloader_uart_read_data();
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Button is not pressed... Executing user Application\n\r");
		bootloader_jump_to_user_app();
	}
	
}

void bootloader_uart_read_data(void)
{
	uint8_t rcv_len = 0;
	while(1)
	{
		memset(bl_rx_buffer, 0, 200);
		/*Read and decode commands receieved from host*/
		
		/*1. Read only one byte from host*/
		HAL_UART_Receive(&huart2, bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];
		
		/*2. Read remaining bytes of the command*/
		HAL_UART_Receive(&huart2, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
		
		/*3. Decode*/
		switch(bl_rx_buffer[1])
		{
			case BL_GET_VER:
				bootloader_handle_getver_cmd(bl_rx_buffer);
			  break;
			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(bl_rx_buffer);
			  break;
			case BL_GET_CID:
				bootloader_handle_getcid_cmd(bl_rx_buffer);
			  break;
			case BL_RDP_STATUS:
				bootloader_handle_rdpstatus_cmd(bl_rx_buffer);
			  break;
			case BL_GO_TO_ADDR:
				bootloader_handle_gotoaddr_cmd(bl_rx_buffer);
			  break;
			case BL_FLASH_ERASE:
				bootloader_handle_flasherase_cmd(bl_rx_buffer);
			case BL_MEM_WRITE:
				bootloader_handle_memwrite_cmd(bl_rx_buffer);
			  break;
			case BL_ENDIS_RW_PROTECT:
				bootloader_handle_endisrwprotect_cmd(bl_rx_buffer);
			  break;
			case BL_MEM_READ:
				bootloader_handle_memread_cmd(bl_rx_buffer);
			  break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handle_readsectorstatus_cmd(bl_rx_buffer);
			  break;
		}
	}
}

/*
 * Code to jump to user application. We assume that
 * the user application is stored in FLASH_SECTOR2_BASE_ADDRESS
 */
void bootloader_jump_to_user_app(void)
{
	/*Function pointer to hold the base address of reset handler of user application*/
	void (*app_reset_handler)(void);
	printmsg("BL_DEBUG_MSG: Jump to user application\n");
	/*1. Configure the MSP by reading value from FLASH_2_BASE_ADDRESS*/
	uint32_t msp_value = *(volatile uint32_t*)FLASH_SECTOR_2;
	printmsg("BL_DEBUG_MSG: MSP Value: %#x\n", msp_value);
	
	/*This function comes from CMSIS*/
	__set_MSP(msp_value);
	
	/*2. Fetch Reset Handler address of user application*/
	uint32_t reset_handler_address = *(uint32_t*)(FLASH_SECTOR2_BASE_ADDRESS + 0x04);
	
	app_reset_handler = (void*)reset_handler_address;
	
	/*3. Jump to reset handler of application*/
	app_reset_handler();
}

/**
  * @brief Print message
  * @retval Includes format specifiers in the argument
  */
void printmsg(char *format, ...)
{
	#ifdef BL_DEBUG_MSG_EN
	char str[80];
	/*Extract the arguments using Virtual Argument APIs*/
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
	#endif
}

/*Bootloader handle functions*/
void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t bl_version;
	
	/*Verify the checksum*/
	printmsg("BL_DEBUG_MSG: Bootloade Handle GETVER command\n");
	/*Total length of command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;
	/*Extract the crc sent by host*/
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer + command_packet_len - 4));
	
	if(!(bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)))
	{
		printmsg("BL_DEBUG_MSG: Checksum Success!!\n");
		
		bootloader_send_ack(bl_rx_buffer[0], 1); /*The length to follow is command specific*/
		bl_version = get_bootloader_version();
		
		printmsg("BL_DEBUG_MSG: BL_VER: %d%#x\n", bl_version, bl_version);
		bootloader_uart_write_data(&bl_version, 1);
	}
	else
	{
		printmsg("BL_DEBUG_MSG: Checksum fail!!\n");
		bootloader_send_nack();
	}
}

void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer)
{
	printmsg("BL_DEBUG_MSG: Bootloader Handle GETHELP command\n");

	/*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    printmsg("BL_DEBUG_MSG:checksum success !!\n");
    bootloader_send_ack(bl_rx_buffer[0],sizeof(supported_commands));
    bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );

	}else
	{
    printmsg("BL_DEBUG_MSG:checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer)
{
	printmsg("BL_DEBUG_MSG: Bootloader Handle GETCID command\n");

	uint16_t bl_cid_num = 0;
	
	/*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\n");
    bootloader_send_ack(bl_rx_buffer[0],2); /*as 2 bytes of chip id will be sent*/
		bl_cid_num = get_mcu_chip_id();  
		printmsg("BL_DEBUG_MSG: MCU id : %d %#x !!\n",bl_cid_num, bl_cid_num);
    bootloader_uart_write_data((uint8_t*)&bl_cid_num,2);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_rdpstatus_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t rdp_level = 0x00;
	printmsg("BL_DEBUG_MSG: Bootloader Handle GETRDP command\n");
	
	/*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\n");
    bootloader_send_ack(bl_rx_buffer[0],1); /*as 2 bytes of chip id will be sent*/
		rdp_level = get_flash_rdp_level();  
		printmsg("BL_DEBUG_MSG: RDP Level : %d %#x !!\n", rdp_level, rdp_level);
    bootloader_uart_write_data(&rdp_level,1);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_gotoaddr_cmd(uint8_t* bl_rx_buffer)
{
  uint32_t go_address=0;
  uint8_t addr_valid = ADDR_VALID;
  uint8_t addr_invalid = ADDR_INVALID;

  printmsg("BL_DEBUG_MSG: Bootloader handle GOTOADDR command\n");

  /*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    printmsg("BL_DEBUG_MSG: checksum success !!\n");

    bootloader_send_ack(bl_rx_buffer[0],1);

		/*extract the go address*/
    go_address = *((uint32_t *)&bl_rx_buffer[2] );
    printmsg("BL_DEBUG_MSG: GO address: %#x\n",go_address);

    if( verify_address(go_address) == ADDR_VALID )
    {
      /*tell host that address is fine*/
      bootloader_uart_write_data(&addr_valid,1);
      
			/* Ensure that the T bit in address is 1 else hardfault exception for ARM cortex M */

      if(!(go_address && 0x00000001))
			{
				go_address += 1;
			}
			
			/*jump to "go_address"*/
      void (*lets_jump)(void) = (void *)go_address;

      printmsg("BL_DEBUG_MSG: Jumping to go address! \n");
            
			lets_jump();
		}
		else
		{
			/*tell host that address is invalid*/
      printmsg("BL_DEBUG_MSG: GO address invalid ! \n");
      bootloader_uart_write_data(&addr_invalid,1);
		}
	}
	else
	{
    printmsg("BL_DEBUG_MSG: checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_flasherase_cmd(uint8_t* bl_rx_buffer)
{
  uint8_t erase_status = 0x00;
  printmsg("BL_DEBUG_MSG: Bootloader handle FLASHERASE command\n");

  /*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * )(bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    printmsg("BL_DEBUG_MSG: checksum success !!\n");
    bootloader_send_ack(bl_rx_buffer[0],1);
    printmsg("BL_DEBUG_MSG: initial_sector : %d  no_ofsectors: %d\n", bl_rx_buffer[2],bl_rx_buffer[3]);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    erase_status = execute_flash_erase(bl_rx_buffer[2] , bl_rx_buffer[3]);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

    printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",erase_status);

    bootloader_uart_write_data(&erase_status,1);
	}
	else
	{
    printmsg("BL_DEBUG_MSG: checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_memwrite_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t write_status = 0x00;
	uint8_t payload_len = bl_rx_buffer[6];

	uint32_t mem_address = *((uint32_t *) ( &bl_rx_buffer[2]) );

  printmsg("BL_DEBUG_MSG: Bootloader handle MEMWRITE command\n");

  /*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * )(bl_rx_buffer+command_packet_len - 4)) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: checksum success !!\n");

    bootloader_send_ack(bl_rx_buffer[0],1);

    printmsg("BL_DEBUG_MSG: mem write address : %#x\n",mem_address);

		if( verify_address(mem_address) == ADDR_VALID )
		{
      printmsg("BL_DEBUG_MSG: valid mem write address\n");

      /*glow the led to indicate bootloader is currently writing to memory*/
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

      /*execute mem write*/
      write_status = execute_mem_write(&bl_rx_buffer[7],mem_address, payload_len);

      /*turn off the led to indicate memory write is over*/
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

      /*Inform host about the status*/
      bootloader_uart_write_data(&write_status,1);
		}
		else
		{
      printmsg("BL_DEBUG_MSG: invalid mem write address\n");
      write_status = ADDR_INVALID;
      /*Inform host that address is invalid*/
      bootloader_uart_write_data(&write_status,1);
		}
	}
	else
	{
    printmsg("BL_DEBUG_MSG: checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_endisrwprotect_cmd(uint8_t* bl_rx_buffer)
{
	uint8_t status = 0x00;
  printmsg("BL_DEBUG_MSG: Bootloader handle ENDISRWPROTECT command\n");

  /*Total length of the command packet*/
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	/*extract the CRC32 sent by the Host*/
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    printmsg("BL_DEBUG_MSG: checksum success !!\n");
    bootloader_send_ack(bl_rx_buffer[0],1);

    status = configure_flash_sector_rw_protection(bl_rx_buffer[2], bl_rx_buffer[3],0);

    printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",status);

    bootloader_uart_write_data(&status,1);
	}
	else
	{
    printmsg("BL_DEBUG_MSG: checksum fail !!\n");
    bootloader_send_nack();
	}
}

void bootloader_handle_readsectorstatus_cmd(uint8_t* bl_rx_buffer)
{
	uint16_t status;
	
	printmsg("BL_DEBUG_MSG: Bootloader handle READSECTOR PROTECTSTATUS command\n");

	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG: checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0],2);
		status = read_OB_rw_protection_status();
		printmsg("BL_DEBUG_MSG: nWRP status: %#x\n",status);
		bootloader_uart_write_data((uint8_t*)&status,2);
	}
	else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
    bootloader_send_nack();
	}
}

/*Bootloader function to send ACK to HOST*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	/*Here we can send two bytes*/
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(&huart2, ack_buf, 2, HAL_MAX_DELAY);
}

/*Bootloader function to send NACK to HOST*/
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(&huart2, &nack, 1, HAL_MAX_DELAY);
}

/*Function to find the CRC of given buffer in pData*/
uint8_t bootloader_verify_crc(uint8_t* pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;
	for(uint32_t i=0; i<len; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	if(uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	else
	{
		return VERIFY_CRC_FAIL;
	}
}

/*Function to get bootloader version*/
uint8_t get_bootloader_version(void)
{
	return (uint8_t)BL_VERSION;
}

/*Wrapper function to send the reply to HOST*/
void bootloader_uart_write_data(uint8_t* pBuffer, uint32_t len)
{
	HAL_UART_Transmit(&huart2, pBuffer, len, HAL_MAX_DELAY);
}

/*Read the chip or device identifier*/
uint16_t get_mcu_chip_id(void)
{
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return cid;
}

/*Read the Read protection level of Flash Memory*/
uint8_t get_flash_rdp_level(void)
{
	uint8_t rdp_status = 0;
	#if 0
	FLASH_OBProgramInitTypeDef ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
	#else
	volatile uint32_t *pOB_addr = (uint32_t*)0x1FFFC000;
	rdp_status = (uint8_t)(*pOB_addr >> 8);
	#endif
	return rdp_status;
}

/*Verify the address the HOST wants bootloader to jump to*/
uint8_t verify_address(uint32_t go_address)
{
	/* The valid addresses to which we can jump are as follows:
	 * 1. System memory
	 * 2. SRAM1 memory
	 * 3. SRAM2 memory 
	 * 4. Backup SRAM memory
	 * 5. External memory
	 */

	if ( go_address >= SRAM1_BASE && go_address <= SRAM2_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
}

/*Function to erase all or some of the sectors of Flash Memory*/
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
  /* There are a total of 8 sectors in STM32F446RE MCU.. sector[0 to 7]
	 * number_of_sector has to be in the range of 0 to 7
	 * if sector_number = 0xff , that means mass erase all sectors!
	 */
	
	/*Create the structure to be sent as an argument to Flash Erase API*/
	FLASH_EraseInitTypeDef flashErase_handle;
	
	uint32_t sectorError;
	HAL_StatusTypeDef status;

	if( number_of_sector > 8 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 7) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
		  /*Calculate the number of sectors to be erased */
			uint8_t remanining_sector = 8 - sector_number;
      if( number_of_sector > remanining_sector)
      {
        number_of_sector = remanining_sector;
      }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number;               /*This is the initial sector*/
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Unlock the flash to get access to the flash registers */
		HAL_FLASH_Unlock();
		
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /*MCU works on this voltage range*/
		status = HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		
		/*Lock the Flash again*/
		HAL_FLASH_Lock();

		return status;
	}
	return INVALID_SECTOR;
}

/*Function to write the contents of Buffer to "mem_address" byte by byte*/
/* Note : 1. Function supports writing to Flash only.
          2. Function does not check whether "mem_address" is a valid address.*/
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
	uint8_t status=HAL_OK;

  /*We have to unlock flash module to get control of registers*/
  HAL_FLASH_Unlock();

  for(uint32_t i = 0 ; i <len ; i++)
  {
    /*Here we program the flash byte by byte*/
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,bl_rx_buffer[i] );
  }

  HAL_FLASH_Lock();

  return status;
}

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
	/* First configure the protection mode
	 * protection_mode = 1 means write protect of the user flash sectors
   * protection_mode = 2 means read/write protect of the user flash sectors
   */

	/*Flash option control register (OPTCR) used to modify Option Bytes*/
  volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	if(disable)
	{
		/*disable all r/w protection on sectors*/

  	/*Option byte configuration unlock*/
		HAL_FLASH_OB_Unlock();

		/*wait till there is no active operation on flash*/
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		/*clear the 31st bit i.e. SPRMOD(default state)*/
		*pOPTCR &= ~(1 << 31);

		/*clear the protection : make all bits belonging to sectors as 1*/
		*pOPTCR |= (0xFF << 16);

		/*Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		*pOPTCR |= ( 1 << 1);

		/*wait till no active operation on flash*/
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();

		return 0;
	}

  if(protection_mode == (uint8_t)1)
  {
		/*Put write protection on the sectors encoded in sector_details argument*/

		/*Option byte configuration unlock*/
		HAL_FLASH_OB_Unlock();

		/*wait till there is no active operation on flash*/
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		/*here we are setting just write protection for the sectors by clearing the 31st bit*/
		*pOPTCR &= ~(1 << 31);

		/*put write protection on sectors*/
		*pOPTCR &= ~ (sector_details << 16);

		/*Set the option start bit (OPTSTRT) in the FLASH_OPTCR register*/
		*pOPTCR |= ( 1 << 1);

		/*wait till there is no active operation on flash*/
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
	}

	else if (protection_mode == (uint8_t)2)
  {
    HAL_FLASH_OB_Unlock();

		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		*pOPTCR |=  (1 << 31);
		*pOPTCR &= ~(0xff << 16);
    *pOPTCR |= (sector_details << 16);

    *pOPTCR |= ( 1 << 1);

		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
  }

	return 0;
}

uint16_t read_OB_rw_protection_status(void)
{
	/*Structure given by ST Flash driver to hold the OB(Option Byte) contents*/
	FLASH_OBProgramInitTypeDef OBInit;

	/*1. Unlock the OB(Option Byte) memory access*/
	HAL_FLASH_OB_Unlock();
	/*2. get the Option Byte configuration details*/
	HAL_FLASHEx_OBGetConfig(&OBInit);
	/*Lock back*/
	HAL_FLASH_Lock();

	/*We are just interested in r/w protection status of the sectors.*/
	return (uint16_t)OBInit.WRPSector;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
