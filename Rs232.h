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

/* Last revision: May 31, 2019 */

/* For more info and how to use this library, visit: http://www.teuniz.net/RS-232/ */


#ifndef rs232_INCLUDED
#define rs232_INCLUDED

#include <stdio.h>
#include <string.h>

#if defined(__linux__) || defined(__FreeBSD__)

#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>

#else

#include <windows.h>

#endif

namespace rs232 {
    int OpenComport(int, int, const char*, int);
    int PollComport(int, unsigned char*, int);
    int SendByte(int, unsigned char);
    int SendBuf(int, unsigned char*, int);
    void CloseComport(int);
    void cputs(int, const char*);
    int IsDCDEnabled(int);
    int IsRINGEnabled(int);
    int IsCTSEnabled(int);
    int IsDSREnabled(int);
    void enableDTR(int);
    void disableDTR(int);
    void enableRTS(int);
    void disableRTS(int);
    void flushRX(int);
    void flushTX(int);
    void flushRXTX(int);
    int GetPortnr(const char*);
}

#endif
