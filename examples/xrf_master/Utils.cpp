/*
 * Utils.cpp
 *
 *  Created on: Oct 9, 2013
 *      Author: Kees Bakker
 */


#include <stddef.h>
#include <stdint.h>
#include <util/crc16.h>


/*
 * \brief Compute CRC16 of a byte buffer
 */
uint16_t crc16_ccitt(uint8_t * buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = _crc_ccitt_update(crc, *buf++);
    }
    return crc;
}
