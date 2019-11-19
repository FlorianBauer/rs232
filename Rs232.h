/*
 ***************************************************************************
 *
 * Author: Teunis van Beelen
 *
 * Copyright (C) 2005 - 2019 Teunis van Beelen
 *
 * Email: teuniz@protonmail.com
 *
 ***************************************************************************
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ***************************************************************************
 */

#ifndef RS232_H
#define RS232_H

#include <cstdint>
#include <cstdlib>

namespace rs232 {

#if defined(__linux__) || defined(__FreeBSD__)

    static constexpr const char* const COMPORTS[] = {
        /* 00 */ "/dev/ttyS0",
        /* 01 */ "/dev/ttyS1",
        /* 02 */ "/dev/ttyS2",
        /* 03 */ "/dev/ttyS3",
        /* 04 */ "/dev/ttyS4",
        /* 05 */ "/dev/ttyS5",
        /* 06 */ "/dev/ttyS6",
        /* 07 */ "/dev/ttyS7",
        /* 08 */ "/dev/ttyS8",
        /* 09 */ "/dev/ttyS9",
        /* 10 */ "/dev/ttyS10",
        /* 11 */ "/dev/ttyS11",
        /* 12 */ "/dev/ttyS12",
        /* 13 */ "/dev/ttyS13",
        /* 14 */ "/dev/ttyS14",
        /* 15 */ "/dev/ttyS15",
        /* 16 */ "/dev/ttyUSB0",
        /* 17 */ "/dev/ttyUSB1",
        /* 18 */ "/dev/ttyUSB2",
        /* 19 */ "/dev/ttyUSB3",
        /* 20 */ "/dev/ttyUSB4",
        /* 21 */ "/dev/ttyUSB5",
        /* 22 */ "/dev/ttyAMA0",
        /* 23 */ "/dev/ttyAMA1",
        /* 24 */ "/dev/ttyACM0",
        /* 25 */ "/dev/ttyACM1",
        /* 26 */ "/dev/rfcomm0",
        /* 27 */ "/dev/rfcomm1",
        /* 28 */ "/dev/ircomm0",
        /* 29 */ "/dev/ircomm1",
        /* 30 */ "/dev/cuau0",
        /* 31 */ "/dev/cuau1",
        /* 32 */ "/dev/cuau2",
        /* 33 */ "/dev/cuau3",
        /* 34 */ "/dev/cuaU0",
        /* 35 */ "/dev/cuaU1",
        /* 36 */ "/dev/cuaU2",
        /* 37 */ "/dev/cuaU3",
    };

#else // windows

    static constexpr const char* const COMPORTS[] = {
        /* 00 */ "\\\\.\\COM1",
        /* 01 */ "\\\\.\\COM2",
        /* 02 */ "\\\\.\\COM3",
        /* 03 */ "\\\\.\\COM4",
        /* 04 */ "\\\\.\\COM5",
        /* 05 */ "\\\\.\\COM6",
        /* 06 */ "\\\\.\\COM7",
        /* 07 */ "\\\\.\\COM8",
        /* 08 */ "\\\\.\\COM9",
        /* 09 */ "\\\\.\\COM10",
        /* 10 */ "\\\\.\\COM11",
        /* 11 */ "\\\\.\\COM12",
        /* 12 */ "\\\\.\\COM13",
        /* 13 */ "\\\\.\\COM14",
        /* 14 */ "\\\\.\\COM15",
        /* 15 */ "\\\\.\\COM16",
        /* 16 */ "\\\\.\\COM17",
        /* 17 */ "\\\\.\\COM18",
        /* 18 */ "\\\\.\\COM19",
        /* 19 */ "\\\\.\\COM20",
        /* 20 */ "\\\\.\\COM21",
        /* 21 */ "\\\\.\\COM22",
        /* 22 */ "\\\\.\\COM23",
        /* 23 */ "\\\\.\\COM24",
        /* 24 */ "\\\\.\\COM25",
        /* 25 */ "\\\\.\\COM26",
        /* 26 */ "\\\\.\\COM27",
        /* 27 */ "\\\\.\\COM28",
        /* 28 */ "\\\\.\\COM29",
        /* 29 */ "\\\\.\\COM30",
        /* 30 */ "\\\\.\\COM31",
        /* 31 */ "\\\\.\\COM32",
    };

#endif

    static constexpr int RS232_PORTNR = sizeof(COMPORTS) / sizeof(COMPORTS[0]);

    int openComport(int, int, const char*, int);
    int pollComport(int, uint8_t*, size_t);
    int sendByte(int, const uint8_t);
    int sendBuf(int, const uint8_t*, size_t);
    void closeComport(int);
    void cputs(int, const char*);
    bool isDcdEnabled(int);
    bool isRingEnabled(int);
    bool isCtsEnabled(int);
    bool isDsrEnabled(int);
    void enableDtr(int);
    void disableDtr(int);
    void enableRts(int);
    void disableRts(int);
    void flushRx(int);
    void flushTx(int);
    void flushRxTx(int);
    int getPortNr(const char*);
}

#endif // RS232_H
