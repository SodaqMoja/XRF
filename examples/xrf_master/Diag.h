#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>

/*
 * \brief Define to switch DIAG on of off
 *
 * Comment or make it a #undef
 */
#define ENABLE_DIAG     1

#ifdef ENABLE_DIAG
#include <SoftwareSerial.h>
extern SoftwareSerial diagport;
#define DIAGPRINT(...)          diagport.print(__VA_ARGS__)
#define DIAGPRINTLN(...)        diagport.println(__VA_ARGS__)
void dumpBuffer(uint8_t * buf, size_t size);

#else
#define DIAGPRINT(...)
#define DIAGPRINTLN(...)
#define dumpBuffer(buf, size)
#endif


#endif //  DIAG_H
