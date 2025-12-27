/*
 * processing.h
 *
 *  Created on: Dec 24, 2025
 *      Author: arthu
 */

#ifndef INC_PROCESSING_H_
#define INC_PROCESSING_H_

#include "main.h"

int extract_nucleoid(const char data[], OtherDevice_t *ts);
int extract_timestamp(const char data[], OtherDevice_t *ts);
int extract_acceleration(const char data[], OtherDevice_t *ts);
int extract_status(const char data[], OtherDevice_t *ts);
uint8_t BCDToDecimal(uint8_t BCD);
uint8_t DecimalToBCD(uint8_t decimal);


#endif /* INC_PROCESSING_H_ */
