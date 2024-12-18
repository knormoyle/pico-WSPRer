///////////////////////////////////////////////////////////////////////////////
//
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https://www.qrz.com/db/r2bdy
//
///////////////////////////////////////////////////////////////////////////////
//
//
//  logutils.h - A set of utilities for logging/debugging.
//
//  DESCRIPTION
//      -
//
//  HOWTOSTART
//      -
//
//  PLATFORM
//      Raspberry Pi pico.
//
//  REVISION HISTORY
//      -
//
//  PROJECT PAGE
//      https://github.com/RPiks/pico-WSPR-tx
//
//  LICENCE
//      MIT License (http://www.opensource.org/licenses/mit-license.php)
//
//  Copyright (c) 2023 by Roman Piksaykin
//  
//  Permission is hereby granted, free of charge,to any person obtaining a copy
//  of this software and associated documentation files (the Software), to deal
//  in the Software without restriction,including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"

#define BUFFER_SIZE 256

static char logBuffer[BUFFER_SIZE] = {0};


/// @brief Buffers the log information to the log buffer
/// @brief it is much faster than direct UART output
/// @param pformat printf style format
/// @param ... argument list to print 
void StampPrintf(const char* pformat, ...)
{
    // background on i/o
    // https://ceworkbench.wordpress.com/2023/01/04/using-the-raspberry-pi-pico-gpio-with-the-c-c-sdk/
    
    static uint32_t sTick = 0;
    // inits the serial port so you can use printf
    if(!sTick) stdio_init_all();

    uint64_t tm_us = to_us_since_boot(get_absolute_time());
    
    const uint32_t tm_day = (uint32_t)(tm_us / 86400000000ULL);
    tm_us -= (uint64_t)tm_day * 86400000000ULL;

    const uint32_t tm_hour = (uint32_t)(tm_us / 3600000000ULL);
    tm_us -= (uint64_t)tm_hour * 3600000000ULL;

    const uint32_t tm_min = (uint32_t)(tm_us / 60000000ULL);
    tm_us -= (uint64_t)tm_min * 60000000ULL;
    
    const uint32_t tm_sec = (uint32_t)(tm_us / 1000000ULL);
    tm_us -= (uint64_t)tm_sec * 1000000ULL;

    char timestamp[64];  //let's create timestamp
    snprintf(timestamp, sizeof(timestamp), "%02lud%02lu:%02lu:%02lu.%06llu [%04lu] ", tm_day, tm_hour, tm_min, tm_sec, tm_us, sTick++);

    va_list argptr;
    va_start(argptr, pformat);
    char message[BUFFER_SIZE];
    vsnprintf(message, sizeof(message), pformat, argptr); //let's format the message 
    va_end(argptr);
    strncat(logBuffer, timestamp, BUFFER_SIZE - strlen(logBuffer) - 1);
    strncat(logBuffer, message, BUFFER_SIZE - strlen(logBuffer) - 1);
    strncat(logBuffer, "\n", BUFFER_SIZE - strlen(logBuffer) - 1);
    
}

/// @brief Outputs the content of the log buffer to stdio (UART and/or USB)
/// @brief Direct output to UART is very slow so we will do it in CPU idle times
/// @brief and not in time critical functions
void DoLogPrint()
{
    if (logBuffer[0] != '\0')
    {
        printf("%s", logBuffer);
        logBuffer[0] = '\0';  // Clear the buffer

    }

}
