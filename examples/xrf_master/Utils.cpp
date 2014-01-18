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
 * \brief Compute CRC16 of a byte buffer (CCITT)
 */
uint16_t crc16_ccitt(uint8_t * buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = _crc_ccitt_update(crc, *buf++);
    }
    return crc;
}

/*
 * \brief Compute CRC16 of a byte buffer (XMODEM)
 */
uint16_t crc16_xmodem(uint8_t * buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc = _crc_xmodem_update(crc, *buf++);
    }
    return crc;
}
