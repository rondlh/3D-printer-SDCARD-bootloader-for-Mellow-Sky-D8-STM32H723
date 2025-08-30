/*----------------------------------------------------------------------------------/
 * Fast MMCv3 / SDv1 / SDv2 in SPI mode control module by Rondlh 2025
 * SD card interfacing based on Simplified Spec Physical Layer V9.10 (Dec 1st 2023)
 *---------------------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "user_diskio_spi.h"
#include "string.h"

// In main.h define:
//   SD_SPI_HANDLE (hspi<x>, x represents a number)
//   SD_CS_Pin (e.g., GPIO_PIN_4)
//   SD_CS_GPIO_Port (e.g., GPIOC)

extern SPI_HandleTypeDef SD_SPI_HANDLE;

/*--------------------------------------------------------------------------*/
/* MMC / SD command reference SPI mode										*/
/*--------------------------------------------------------------------------*/
#define   CMD0			 (0) // GO_IDLE_STATE
#define   CMD1			 (1) // SEND_OP_COND (MMC)
#define	ACMD41	   (0x80+41) // SEND_OP_COND (SDC)
#define   CMD8			 (8) // SEND_IF_COND
#define   CMD9			 (9) // SEND_CSD
#define  CMD10			(10) // SEND_CID
#define  CMD12			(12) // STOP_TRANSMISSION
#define  CMD13			(13) // SEND_STATUS
#define ACMD13 	   (0x80+13) // SD_STATUS (SDC)
#define  CMD16			(16) // SET_BLOCKLEN
#define  CMD17			(17) // READ_SINGLE_BLOCK
#define  CMD18			(18) // READ_MULTIPLE_BLOCK
#define  CMD23			(23) // SET_BLOCK_COUNT (MMC)
#define	ACMD23	   (0x80+23) // SET_WR_BLK_ERASE_COUNT (SDC)
#define  CMD24			(24) // WRITE_BLOCK
#define  CMD25			(25) // WRITE_MULTIPLE_BLOCK
#define  CMD32			(32) // ERASE_ER_BLK_START
#define  CMD33			(33) // ERASE_ER_BLK_END
#define  CMD38			(38) // ERASE
#define  CMD41			(41) // SEND_OP_COND (ACMD)
#define  CMD55			(55) // APP_CMD
#define  CMD58			(58) // READ_OCR

// MMC card type flags (MMC_GET_TYPE)
#define CT_MMC			0x01 // MMC version 3
#define CT_SD1			0x02 // SD version 1
#define CT_SD2			0x04 // SD version 2
#define CT_SDC (CT_SD1|CT_SD2) // SD
#define CT_BLOCK		0x08 // Block addressing

#define SECTOR_SIZE		512  // Sector size of the SD/MMC card

static volatile DSTATUS DiskStatus = STA_NOINIT;

static BYTE CardType;	// b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing

// This is a stripped down version of:
// HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
void HAL_SPI_TransmitReceive_fast(const uint8_t *pTxData, uint8_t *pRxData, uint16_t count)
{
	uint32_t rx_count = count;
	uint32_t tx_count = count;
	static volatile uint8_t dummy;

	volatile uint8_t *TXDR = (uint8_t*) &(SD_SPI_HANDLE.Instance->TXDR);
	volatile uint8_t *RXDR = (uint8_t*) &(SD_SPI_HANDLE.Instance->RXDR);
	volatile uint32_t *SR = &(SD_SPI_HANDLE.Instance->SR);

	__HAL_SPI_ENABLE(&SD_SPI_HANDLE); // Required
	SET_BIT(SD_SPI_HANDLE.Instance->CR1, SPI_CR1_CSTART); // Required

	while (tx_count || rx_count)
	{
		// Check the TXP flag
		if (tx_count && (*SR & SPI_FLAG_TXE))
		{
			if (pTxData)
				*TXDR = *pTxData++;
			else
				*TXDR = 0xFF;
			tx_count--;
		}

		if (rx_count && (*SR & SPI_FLAG_RXNE))
		{
			if (pRxData)
			  *pRxData++ = *RXDR;
			else
			  dummy = *RXDR;
			rx_count--;
		}
	}

	__HAL_SPI_CLEAR_TXTFFLAG(&SD_SPI_HANDLE); // Not Required
	__HAL_SPI_DISABLE(&SD_SPI_HANDLE);		  // Not Required
	UNUSED(dummy);
}

#if _USE_WRITE
/*----------------------------------------------------------------------*/
/* Transmit data to the card											*/
/*----------------------------------------------------------------------*/
static void transmit_mmc(const BYTE *buf, UINT count)
{
	HAL_SPI_TransmitReceive_fast(buf, NULL, count);
}
#endif // _USE_WRITE

/*----------------------------------------------------------------------*/
/* Receive 1 byte from the MMC											*/
/*----------------------------------------------------------------------*/
static BYTE receive_mmc1()
{
	BYTE rxdata;
	HAL_SPI_TransmitReceive_fast(NULL, &rxdata, 1);
	return rxdata;
}

/*----------------------------------------------------------------------*/
/* Wait for card ready													*/
/*----------------------------------------------------------------------*/
// 1:Ready, 0:Timeout
static int wait_ready(UINT timeout)
{
	BYTE data = 0;
	uint32_t start = HAL_GetTick();

	while ((data != 0xFF) && (HAL_GetTick() - start < timeout))
	{
		data = receive_mmc1();
	}

	return (data == 0xFF);
}

/*----------------------------------------------------------------------*/
/* De-select the card and release the SPI bus							*/
/*----------------------------------------------------------------------*/
static void spi_deselect(void)
{
#ifdef SD_CS_GPIO_Port
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
#else
	#warning Make sure SD_CS_GPIO_Port and SD_CS_Pin are defined when not using SPI Hardware NSS
#endif

	receive_mmc1(); // Dummy clock to force DO hi-z for multiple slave SPI
}

/*----------------------------------------------------------------------*/
/* Select card and wait for card ready									*/
/*----------------------------------------------------------------------*/
// 1:OK, 0:Timeout
static int spi_select(void)
{
#ifdef SD_CS_GPIO_Port
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
#else
	#warning Make sure SD_CS_GPIO_Port and SD_CS_Pin are defined when not using SPI Hardware NSS
#endif
	receive_mmc1(); // Dummy clock (force DO enabled)

	if (wait_ready(500))
		return 1; // OK

	spi_deselect();
	return 0; // Timeout
}

/*----------------------------------------------------------------------*/
/* Receive a data sector from the MMC									*/
/*----------------------------------------------------------------------*/
// 1:OK, 0:Error
static int receive_datablock(BYTE *buf, UINT count)
{
	BYTE data[2];

	do { // Wait for DataStart token
		data[0] = receive_mmc1();
	} while (data[0] == 0xFF);

	// Check for invalid data
	if (data[0] != 0xFE)
		return 0; // Error

	HAL_SPI_TransmitReceive_fast(NULL, buf, count); // Receive the data block
	HAL_SPI_TransmitReceive_fast(NULL, data, 2);	// Read CRC

	return 1; // OK
}

/*----------------------------------------------------------------------*/
/* Send a data sector to the MMC										*/
/*----------------------------------------------------------------------*/
// 1:OK, 0:Failed
#if _USE_WRITE
static int transmit_datablock(const BYTE *buf, BYTE token)
{
	BYTE data[2];

	if (!wait_ready(500))
		return 0;

	data[0] = token;
	transmit_mmc(data, 1);		// Transmit token
	if (token != 0xFD)			// Is it data token?
		{
		transmit_mmc(buf, SECTOR_SIZE); // Transmit 1 sector of data to the MMC
		HAL_SPI_TransmitReceive_fast(NULL, data, 2); // Dummy CRC
		data[0] = receive_mmc1(); 		// Receive data response

		if ((data[0] & 0x1F) != 0x05)
			return 0; // Failed, data not accepted
	}
	return 1; // OK
}

#endif // _USE_WRITE

/*----------------------------------------------------------------------*/
/* Receive 1 byte from the MMC											*/
/*----------------------------------------------------------------------*/
static BYTE transmit_mmc1(BYTE txdata)
{
	BYTE rxdata;
	//HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &txdata, &rxdata, 1, 100);
	HAL_SPI_TransmitReceive_fast(&txdata, &rxdata, 1);
	return rxdata;
}

/*----------------------------------------------------------------------*/
/* Send a command to the MMC											*/
/*----------------------------------------------------------------------*/
// Returns command response R1 (bit7==1:Send failed)
static BYTE send_cmd(BYTE cmd, DWORD arg)
{
	BYTE n, result;

	if (cmd & 0x80) // ACMD<n> is the command sequence of CMD55-CMD<n>
	{
		cmd &= 0x7F;
		result = send_cmd(CMD55, 0); // Command: APP_CMD
		if (result > 1)
			return result;
	}

	// Select the card and wait for ready except to stop multiple block read
	if (cmd != CMD12) // Command: STOP_TRANSMISSION
	{
		spi_deselect();
		if (!spi_select())
			return 0xFF;
	}

	// Send a command
	transmit_mmc1(0x40 | cmd);			// Start + command index
	transmit_mmc1((BYTE)(arg >> 24));	// Argument[31..24]
	transmit_mmc1((BYTE)(arg >> 16));	// Argument[23..16]
	transmit_mmc1((BYTE)(arg >>  8));	// Argument[15..8]
	transmit_mmc1((BYTE) arg);			// Argument[7..0]
	n = 0x01;							// Dummy CRC + Stop

	if (cmd == CMD0)					// Valid CRC for CMD0(0)
		n = 0x95;
	else
	if (cmd == CMD8)					// Valid CRC for CMD8(0x1AA)
		n = 0x87;

	transmit_mmc1(n);

	// Receive command response
	if (cmd == CMD12) // Skip a stuff byte when stop reading
		result = receive_mmc1();

	// Wait for a valid response (max 10 attempts)
	n = 10;
	do {
		result = receive_mmc1();
	} while ((result & 0x80) && --n);

	return result; // Returns the response value
}

/*----------------------------------------------------------------------*/
/* Public FATFs Functions (wrapped in user_diskio.c)					*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/* Get disk status														*/
/*----------------------------------------------------------------------*/
DSTATUS USER_SPI_status(BYTE drive) // Drive number, should be 0
{
	if (drive) // Supports only drive 0
		return STA_NOINIT;

	// Check if the card is initialized
	if (!(DiskStatus & STA_NOINIT))
	{
		if (send_cmd(CMD13, 0))	// Command: SEND_STATUS
			DiskStatus = STA_NOINIT;

		receive_mmc1(); // Get next byte of R2
		spi_deselect();
	}

	return DiskStatus;
}

/*----------------------------------------------------------------------*/
/* Initialize disk drive												*/
/*----------------------------------------------------------------------*/
DSTATUS USER_SPI_initialize(BYTE drive) // Physical drive number, should be 0
{
	BYTE n, ty, cmd, buf[4];
	UINT counter;

	if (drive) // Only drive 0 is supported
		return STA_NOINIT;

	__HAL_SPI_ENABLE(&SD_SPI_HANDLE);
	
	if (DiskStatus & STA_NODISK)			// Is a card present?
		return DiskStatus;

	// Store current SPI speed
	uint16_t oldspeed =	(SD_SPI_HANDLE.Instance->CR1 & SPI_BAUDRATEPRESCALER_256);

	// Switch to slow speed (100-400KHz) to enter native operating mode
	SD_SPI_HANDLE.Instance->CR1 =
		(SD_SPI_HANDLE.Instance->CR1 & ~SPI_BAUDRATEPRESCALER_256) | SPI_BAUDRATEPRESCALER_256;

	for (n = 10; n; n--) // Generate 80 dummy clocks at low speed (74 clock are required)
		receive_mmc1();

	// Restore previous (fast) speed
	SD_SPI_HANDLE.Instance->CR1 =
		(SD_SPI_HANDLE.Instance->CR1 & ~SPI_BAUDRATEPRESCALER_256) | oldspeed;

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) // Enter idle state
	{
		if (send_cmd(CMD8, 0x1AA) == 1) // Is it SDv2?
		{	// Get R7 (32 bits)
			HAL_SPI_TransmitReceive_fast(NULL, buf, 4);
			if ((buf[2] == 0x01) && (buf[3] == 0xAA))
			{ // The card can use a Vcc range of 2.7-3.6V

				for (counter = 1000; counter; counter--) // 1 second timeout, usually needs 10ms
				{
					if (send_cmd(ACMD41, 1UL << 30) == 0) // Command: SEND_OP_COND (SDC)
						break;
					HAL_Delay(1);
				}

				if (!send_cmd(CMD58, 0)) // Check CCS bit in the OCR
				{
					HAL_SPI_TransmitReceive_fast(NULL, buf, 4);
					ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // Card ID SDv2
				}
			}
		}
		else // SDv1 or MMCv3, not SDv2
		{
			if (send_cmd(ACMD41, 0) <= 1) // Is it SDv1 or MMC?
			{
				ty = CT_SD1; // SDv1 (ACMD41(0))
				cmd = ACMD41;
			}
			else // MMCv3
			{
				ty = CT_MMC; // MMCv3 (CMD1(0))
				cmd = CMD1;
			}

			for (counter = 1000; counter; counter--) // 1 second timeout for leaving idle state
			{
				if (send_cmd(cmd, 0) == 0)
					break;
				HAL_Delay(1);
			}

			if (!counter || send_cmd(CMD16, SECTOR_SIZE)) // Command: SET_BLOCKLEN 512
				ty = 0;
		}
	}
	CardType = ty; // Card type

	if (ty)						// OK
		DiskStatus &= ~STA_NOINIT;	// Clear STA_NOINIT flag
	 else						// Failed
		DiskStatus = STA_NOINIT;
	spi_deselect();

	return DiskStatus;
}

/*----------------------------------------------------------------------*/
/* Read sector(s)														*/
/*----------------------------------------------------------------------*/
DRESULT USER_SPI_read(BYTE drive,	// Physical drive number, should be 0
					  BYTE *buf,	// Pointer to the data buffer to store the data
					  DWORD sector,	// Start sector number (LBA)
					  UINT count)	// Number of sectors to read (1..128)
{
	if (drive || !count)		// Check drive and count
		return RES_PARERR;

	if (DiskStatus & STA_NOINIT) // Check if card is ready
		return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) // Convert LBA to byte address
		sector = sector * SECTOR_SIZE;

	if (count == 1) // Command: READ_SINGLE_BLOCK
	{
		if ((send_cmd(CMD17, sector) == 0) && receive_datablock(buf, SECTOR_SIZE)) // Command: READ_SINGLE_BLOCK
			count = 0; // Done
	}
	else // Multiple sector read
	{
		if (send_cmd(CMD18, sector) == 0) // Command: READ_MULTIPLE_BLOCK
		{
			do
			{
				if (!receive_datablock(buf, SECTOR_SIZE))
					break;
				buf += SECTOR_SIZE;
			} while (--count);

			send_cmd(CMD12, 0); // Command: STOP_TRANSMISSION
		}
	}

	spi_deselect();

	return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE
/*----------------------------------------------------------------------*/
/* Write sector(s)														*/
/*----------------------------------------------------------------------*/
DRESULT USER_SPI_write(BYTE drive,		// Physical drive number, should be 0
					   const BYTE *buf,	// Pointer to the data to be written
					   DWORD sector,	// Start sector number (LBA)
					   UINT count)		// Number of sectors to write (1..128)
{
	if (drive || !count)		// Check parameters
		return RES_PARERR;

	if (DiskStatus & STA_NOINIT) // Check drive status
		return RES_NOTRDY;

	if (DiskStatus & STA_PROTECT) // Check write protect
		return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) // LBA ==> BA conversion (byte addressing cards)
		sector = sector * SECTOR_SIZE; // Convert LBA to byte address

	if (count == 1) // Single sector write
	{
		if ((send_cmd(CMD24, sector) == 0) && transmit_datablock(buf, 0xFE)) // Command: WRITE_BLOCK
			count = 0;
	}
	else // Multiple sector write
	{
		if (CardType & CT_SDC)
			send_cmd(ACMD23, count); // Command: SET_WR_BLK_ERASE_COUNT

		if (send_cmd(CMD25, sector) == 0) // Command: WRITE_MULTIPLE_BLOCK
		{
			do {
				if (!transmit_datablock(buf, 0xFC))
					break;
				buf += SECTOR_SIZE;
			} while (--count);

			if (!transmit_datablock(0, 0xFD)) // STOP_TRAN token
				count = 1;
		}
	}
	spi_deselect();

	return count ? RES_ERROR : RES_OK;
}

#endif // _USE_WRITE

#if _USE_IOCTL
/*----------------------------------------------------------------------*/
/* Miscellaneous Functions												*/
/*----------------------------------------------------------------------*/
DRESULT USER_SPI_ioctl(BYTE drive, BYTE cmd, void *buf)
{
	DRESULT res;
	BYTE n, csd[16];
	DWORD *dp, st, ed, csize;

	if (drive) // Only drive 0 is supported
		return RES_PARERR;

	if (DiskStatus & STA_NOINIT)  // Check if drive is ready
		return RES_NOTRDY;

	res = RES_ERROR;
	switch (cmd)
	{
		case CTRL_SYNC:	// Wait for internal write to complete
			if (spi_select())
				res = RES_OK;
		break;

		case GET_SECTOR_COUNT: // Get drive capacity in units of sectors (DWORD)
		if ((send_cmd(CMD9, 0) == 0) && receive_datablock(csd, 16)) // Command: SEND_CSD
		{
			if ((csd[0] >> 6) == 1) // SDC version 2.00
			{
				csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(DWORD*)buf = csize << 10;
			}
			else // SDC version 1.XX or MMC version 3
			{
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD*)buf = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;

		case GET_BLOCK_SIZE: // Get erase block size in units of sectors (DWORD)
			if (CardType & CT_SD2)
			{ // SDC version 2.00
				if (send_cmd(ACMD13, 0) == 0) // Command: Read SD status
				{
					receive_mmc1();
					if (receive_datablock(csd, 16)) // Read partial block
					{
						for (n = 48; n; n--)
							receive_mmc1(); // Purge trailing data

						*(DWORD*)buf = 16UL << (csd[10] >> 4);
						res = RES_OK;
					}
				}
			}
			else
			{ // SDC version 1.XX or MMC
				if ((send_cmd(CMD9, 0) == 0) && receive_datablock(csd, 16)) // Command: SEND_CSD
				{
					if (CardType & CT_SD1) // SDC version 1.XX
						*(DWORD*)buf = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
					else // MMC
						*(DWORD*)buf = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
					res = RES_OK;
				}
			}
		break;

		case CTRL_TRIM : // Erase a block of sectors

			if (!(CardType & CT_SDC)) // Check if the card is SDC
				break;

			if (USER_SPI_ioctl(drive, MMC_GET_CSD, csd)) // Get CSD
				break;

			if (!(csd[0] >> 6) && !(csd[10] & 0x40)) // Check if sector erase can be applied to the card
				break;

			dp = buf;
			st = dp[0];
			ed = dp[1]; // Load sector block

			if (!(CardType & CT_BLOCK))
			{
				st = st * SECTOR_SIZE;
				ed = ed * SECTOR_SIZE;
			}

			if ((send_cmd(CMD32, st) == 0) && // Erase sector block
				(send_cmd(CMD33, ed) == 0) &&
				(send_cmd(CMD38,  0) == 0) &&
					wait_ready(20000))
				res = RES_OK;
		break;

		default:
			res = RES_PARERR;
	}

	spi_deselect();

	return res;
}

#endif //_USE_IOCTL
