/*
 * communication.h
 *
 *  Created on: Dec 25, 2025
 *      Author: arthu
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_


// FRAM
void Init_FRAM(void);
void Write_FRAM(uint32_t address, const void *data, uint16_t size);
void Read_FRAM(uint32_t address, void *data, uint16_t size);
void StoreInFRAM(OtherDevice_t *ts);
void FRAM_ClearRange(uint32_t address, uint32_t length);

// RTC
void Init_RTC(void);
void Set_RTC(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t years);
void Read_RTC(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *day, uint8_t *date, uint8_t *month, uint8_t *years);



#endif /* INC_COMMUNICATION_H_ */
