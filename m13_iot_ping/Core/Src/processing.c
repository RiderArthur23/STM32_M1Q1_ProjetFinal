/*
 * processing.c
 *
 *  Created on: Dec 24, 2025
 *      Author: arthu
 */


#include "main.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

int extract_nucleoid(const char data[], OtherDevice_t *ts)
{
    const char *p = strstr(data, "\"id\":");
    if (!p) return 0;

    p = strstr(p, "nucleo-");
    if (!p) return 0;

    ts->NucleoID = (uint8_t)strtoul(p + 7, NULL, 10);
    return 1;
}

int extract_timestamp(const char data[], OtherDevice_t *ts)
{
	const char *p = strstr(data, "\"timestamp\"");
	if (!p) return 0;

	ts->year = (uint8_t)((p[16] - '0') * 10 + (p[17] - '0'));
	ts->mon  = (uint8_t)((p[19] - '0') * 10 + (p[20] - '0'));
	ts->mday = (uint8_t)((p[22] - '0') * 10 + (p[23] - '0'));
	ts->hour = (uint8_t)((p[25] - '0') * 10 + (p[26] - '0'));
	ts->min  = (uint8_t)((p[28] - '0') * 10 + (p[29] - '0'));
	ts->sec  = (uint8_t)((p[31] - '0') * 10 + (p[32] - '0'));

    /*
    sscanf(p, "\"timestamp\": \"%hu-%02hhu-%02hhuT%02hhu:%02hhu:%02hhuZ\"",
        &AllYear,
        &ts->mon,
        &ts->mday,
        &ts->hour,
        &ts->min,
        &ts->sec);
    ts->year = (uint8_t)(AllYear - 2000);
	*/
    return 1;
}


int extract_acceleration(const char data[], OtherDevice_t *ts)
{
    const char *p = strstr(data, "\"acceleration\":");
    if (!p) return 0;

    if (sscanf(p,
        "\"acceleration\": {\"x\": %f,\"y\": %f,\"z\": %f}",
        &ts->LastHighestValue_x,
        &ts->LastHighestValue_y,
        &ts->LastHighestValue_z) == 3) {
        return 1;
    }

    return 0;
}


int extract_status(const char data[], OtherDevice_t *ts)
{
    const char *p = strstr(data, "\"status\":");
    if (!p) return 0;

    p = strchr(p, ':');
    if (!p) return 0;

    p = strchr(p, '"');
    if (!p) return 0;

    p++;

    if (strncmp(p, "normal", 6) == 0) {ts->status = 0;}
    else {ts->status = 1;}

    return 1;
}


uint8_t DecimalToBCD(uint8_t decimal)
{
	uint8_t d_decimal = decimal / 10;
	uint8_t u_decimal = (decimal - (d_decimal * 10));
	uint8_t BCD = (d_decimal << 4) | (u_decimal & 0xF);
	return BCD;
}

uint8_t BCDToDecimal(uint8_t BCD)
{
	uint8_t d_BCD = BCD >> 4;
	uint8_t u_BCD = BCD & 0x0F;
	uint8_t Decimal = (d_BCD * 10) + u_BCD;
	return Decimal;
}
