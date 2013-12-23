/*
 * Utils.h
 *
 *  Created on: Oct 9, 2013
 *      Author: Kees Bakker
 *
 * This is a collection of miscellaneous functions.
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>

uint16_t crc16_ccitt(uint8_t * buf, size_t len);

static inline bool isTimedOut(uint32_t ts)
{
  return (long)(millis() - ts) >= 0;
}


#endif /* UTILS_H_ */
