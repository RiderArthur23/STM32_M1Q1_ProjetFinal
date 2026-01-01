/*
 * communication.c
 *
 *  Created on: Dec 25, 2025
 *      Author: arthu
 */

#include "processing.h"
#include "main.h"
#include "cmsis_os.h"
#include "os_resources.h"	// Use for mutex to not let every file who use main.h to have access to the rtos lib

#define RTC_adr 			0xD0
#define	RTC_adr_seconds		0x00
#define	RTC_adr_minutes		0x01
#define RTC_adr_TCH2		0x08
#define	RTC_adr_CFG2		0x09

#define FRAM_opcode_WREN	0x06
#define FRAM_opcode_write	0x02
#define FRAM_opcode_read	0x03
#define test_adr			0xFF


/*************		FRAM	*************************/
/*
 * As we know, the FRAM can store 512 kB.
 * Therefore, I decided to divide the FRAM into separate memory areas.
 *
 * 1 -> The structures that contain all the data from the other devices occupy a maximum of 24 bytes each.
 *      They are stored between addresses 256 kB and 257.25 kB. (50 devices)
 *      Each structure is allocated an area of 30 bytes to ensure that there are no overlaps
 *      between two consecutive structures.
 *      As a result, the first stored structure starts at address 256 kB and ends at 256024.
 */

void Init_FRAM(void)
{
	// Dummy message
	uint8_t dummy = 0xFF;
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi4, &dummy, 1, 1000);
	osDelay(1);
}

void Write_FRAM(uint32_t address, const void *data, uint16_t size)
{

	osMutexWait(SPI4MutexHandle, osWaitForever);

	// Send the opcode to enable the writting
	uint8_t tdata_init = FRAM_opcode_WREN;
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, &tdata_init, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

	// Send the data
	uint8_t adr[] = {FRAM_opcode_write, (address >> 16),  (address >> 8), address};
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, adr, 4, 1000);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)data, size, 1000);
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

	osMutexRelease(SPI4MutexHandle);
}

void Read_FRAM(uint32_t address, void *data, uint16_t size)
{
	osMutexWait(SPI4MutexHandle, osWaitForever);

	uint8_t adr[] = {FRAM_opcode_read, (address >> 16),  (address >> 8), address};
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, adr, 4, 1000);
	HAL_SPI_Receive(&hspi4, (uint8_t *)data, size, 1000);
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

	osMutexRelease(SPI4MutexHandle);
}


// To store the received data, we first check every 30th byte starting at the 256 kB address,
// as explained at the beginning of the FRAM section, because the first byte of each structure
// contains the device ID, as defined in main.h.
// If the ID is not yet known, the structure is stored after all previously analyzed ones.
// If the same ID as the one being analyzed is found, the data already stored in FRAM
// is erased and replaced with the new data.

#define MaxExternalDevices      50
#define BytesPerExternalDevices  30
#define StartAddressForExternalDevices 256000


void StoreExternDeviceInFRAM(OtherDevice_t *ts)
{
    for (uint8_t i = 0; i < MaxExternalDevices; i++) {

        uint32_t address = StartAddressForExternalDevices + (i * BytesPerExternalDevices);
        uint8_t DeviceFromFRAM = 0;

        Read_FRAM(address, &DeviceFromFRAM, 1);

        if (DeviceFromFRAM == ts->NucleoID) {
            log_message("[SERVER - FRAM] ID known, overwrite\r\n");
            Write_FRAM(address, ts, sizeof(OtherDevice_t));
            osDelay(5);
            return;
        }
        else if (DeviceFromFRAM == 0) {
            log_message("[SERVER - FRAM] New ID, store\r\n");
            Write_FRAM(address, ts, sizeof(OtherDevice_t));
            osDelay(1);
            return;
        }
    }

    log_message("[SERVER - FRAM] FRAM full, no space available\r\n");
}


// Here the adress for local values goes from 100.000 to 100.250

#define MaxLocalValues 10
#define BytesPerLocalValues 25
#define StartAdressForLocalValues 100000

int CheckIfHigher(LocalValue_t *ts)
{
    for (uint8_t i = 0; i < MaxLocalValues; i++)
    {
        uint32_t address = StartAdressForLocalValues + (i * BytesPerLocalValues);

        LocalValue_t stored;
        Read_FRAM(address, &stored, sizeof(LocalValue_t));

        float new_max = fmaxf(ts->Value_x, fmaxf(ts->Value_y, ts->Value_z));

        float stored_max = fmaxf(stored.Value_x, fmaxf(stored.Value_y, stored.Value_z));

        if (new_max > stored_max)
        {
            Write_FRAM(address, ts, sizeof(LocalValue_t));
            log_message("Nouvelle valeur RMS plus elevee stockee en FRAM\r\n");
            return 1;
        }
    }

    log_message("Valeur RMS insuffisante, non stockee\r\n");
    return 0;
}




void FRAM_ClearRange(uint32_t address, uint32_t length)
{
    uint8_t zero = 0;

    for (uint32_t i = 0; i < length; i++) {
        Write_FRAM(address + i, &zero, 1);
    }
}




/*************		RTC		*************************/

void Init_RTC(void)
{
	// Init the super capacitor
	uint8_t tdata_init[] = {RTC_adr_TCH2, 0x20, 0x45};
	HAL_I2C_Master_Transmit(&hi2c2, RTC_adr, tdata_init, 3, 1000);
}

void Set_RTC(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t years)
{
	osMutexWait(I2C2MutexHandle, osWaitForever);

	// Convert decimal values to BCD
	uint8_t BCD_seconds = DecimalToBCD(seconds);
	uint8_t BCD_minutes = DecimalToBCD(minutes);
	uint8_t BCD_hours = DecimalToBCD(hours);
	uint8_t BCD_day = DecimalToBCD(day);
	uint8_t BCD_date = DecimalToBCD(date);
	uint8_t BCD_month = DecimalToBCD(month);
	uint8_t BCD_years = DecimalToBCD(years);

	// Sent all the BCD values to the RTC
	uint8_t tdata[] = {RTC_adr_seconds, BCD_seconds, BCD_minutes, BCD_hours, BCD_day, BCD_date, BCD_month, BCD_years};
	HAL_I2C_Master_Transmit(&hi2c2, RTC_adr, tdata, sizeof(tdata), 1000);

	osMutexRelease(I2C2MutexHandle);
}


/* Using this function has to be like :
 * uint8_t seconds, minutes, hours, ... ;
 * Read_RTC(&seconds, &minutes, &hours, ...);
 */
void Read_RTC(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *day, uint8_t *date, uint8_t *month, uint8_t *years)
{
	osMutexWait(I2C2MutexHandle, osWaitForever);

    uint8_t tdata[] = {RTC_adr_seconds};
    uint8_t rdata[7] = {0};

    HAL_I2C_Master_Transmit(&hi2c2, RTC_adr, tdata, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c2, RTC_adr, rdata, 7, 1000);

    *seconds = BCDToDecimal(rdata[0] & 0x7F);  // Need to mask the non-used bit
    *minutes = BCDToDecimal(rdata[1] & 0x7F);
    *hours   = BCDToDecimal(rdata[2] & 0x3F);

    *day   = BCDToDecimal(rdata[3] & 0x07);
    *date  = BCDToDecimal(rdata[4] & 0x3F);
    *month = BCDToDecimal(rdata[5] & 0x1F);
    *years = BCDToDecimal(rdata[6]);

    osMutexRelease(I2C2MutexHandle);
}
