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

#include <iostream>
#include <cstring>
#include "Rs232.h"

using namespace rs232;

#if defined(__linux__) || defined(__FreeBSD__)   /* Linux & FreeBSD */

#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>

int cPort[RS232_PORTNR];
struct termios newPortSettings;
struct termios oldPortSettings[RS232_PORTNR];

int rs232::openComport(int comportNumber, int baudrate, const char* mode, int flowctrl) {
    int baudr;
    int status;
    int error;

    if ((comportNumber >= RS232_PORTNR) || (comportNumber < 0)) {
        std::cout << "illegal comport number\n";
        return (1);
    }

    switch (baudrate) {
        case 50: baudr = B50;
            break;
        case 75: baudr = B75;
            break;
        case 110: baudr = B110;
            break;
        case 134: baudr = B134;
            break;
        case 150: baudr = B150;
            break;
        case 200: baudr = B200;
            break;
        case 300: baudr = B300;
            break;
        case 600: baudr = B600;
            break;
        case 1200: baudr = B1200;
            break;
        case 1800: baudr = B1800;
            break;
        case 2400: baudr = B2400;
            break;
        case 4800: baudr = B4800;
            break;
        case 9600: baudr = B9600;
            break;
        case 19200: baudr = B19200;
            break;
        case 38400: baudr = B38400;
            break;
        case 57600: baudr = B57600;
            break;
        case 115200: baudr = B115200;
            break;
        case 230400: baudr = B230400;
            break;
        case 460800: baudr = B460800;
            break;
        case 500000: baudr = B500000;
            break;
        case 576000: baudr = B576000;
            break;
        case 921600: baudr = B921600;
            break;
        case 1000000: baudr = B1000000;
            break;
        case 1152000: baudr = B1152000;
            break;
        case 1500000: baudr = B1500000;
            break;
        case 2000000: baudr = B2000000;
            break;
        case 2500000: baudr = B2500000;
            break;
        case 3000000: baudr = B3000000;
            break;
        case 3500000: baudr = B3500000;
            break;
        case 4000000: baudr = B4000000;
            break;
        default:
            std::cout << "invalid baudrate\n";
            return (1);
    }

    int cbits = CS8;
    int cpar = 0;
    int ipar = IGNPAR;
    int bstop = 0;

    if (strlen(mode) != 3) {
        std::cout << "invalid mode \"" << mode << "\"\n";
        return (1);
    }

    switch (mode[0]) {
        case '8': cbits = CS8;
            break;
        case '7': cbits = CS7;
            break;
        case '6': cbits = CS6;
            break;
        case '5': cbits = CS5;
            break;
        default:
            std::cout << "invalid number of data-bits '" << static_cast<char> (mode[0]) << "'\n";
            return (1);
    }

    switch (mode[1]) {
        case 'N':
        case 'n':
            cpar = 0;
            ipar = IGNPAR;
            break;
        case 'E':
        case 'e':
            cpar = PARENB;
            ipar = INPCK;
            break;
        case 'O':
        case 'o':
            cpar = (PARENB | PARODD);
            ipar = INPCK;
            break;
        default:
            std::cout << "invalid parity '" << static_cast<char> (mode[1]) << "'\n";
            return (1);
    }

    switch (mode[2]) {
        case '1': bstop = 0;
            break;
        case '2': bstop = CSTOPB;
            break;
        default:
            std::cout << "invalid number of stop bits '" << static_cast<char> (mode[2]) << "'\n";
            return (1);
    }

    /*
    http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html
    http://man7.org/linux/man-pages/man3/termios.3.html
     */

    cPort[comportNumber] = open(COMPORTS[comportNumber], O_RDWR | O_NOCTTY | O_NDELAY);
    if (cPort[comportNumber] == -1) {
        std::cerr << "unable to open comport ";
        return (1);
    }

    /* lock access so that another process can't also use the port */
    if (flock(cPort[comportNumber], LOCK_EX | LOCK_NB) != 0) {
        close(cPort[comportNumber]);
        std::cerr << "Another process has locked the comport.";
        return (1);
    }

    error = tcgetattr(cPort[comportNumber], oldPortSettings + comportNumber);
    if (error == -1) {
        close(cPort[comportNumber]);
        flock(cPort[comportNumber], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "unable to read portsettings ";
        return (1);
    }
    memset(&newPortSettings, 0, sizeof (newPortSettings)); /* clear the new struct */

    newPortSettings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
    if (flowctrl) {
        newPortSettings.c_cflag |= CRTSCTS;
    }
    newPortSettings.c_iflag = ipar;
    newPortSettings.c_oflag = 0;
    newPortSettings.c_lflag = 0;
    newPortSettings.c_cc[VMIN] = 0; /* block untill n bytes are received */
    newPortSettings.c_cc[VTIME] = 0; /* block untill a timer expires (n * 100 mSec.) */

    cfsetispeed(&newPortSettings, baudr);
    cfsetospeed(&newPortSettings, baudr);

    error = tcsetattr(cPort[comportNumber], TCSANOW, &newPortSettings);
    if (error == -1) {
        tcsetattr(cPort[comportNumber], TCSANOW, oldPortSettings + comportNumber);
        close(cPort[comportNumber]);
        flock(cPort[comportNumber], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "unable to adjust portsettings ";
        return (1);
    }

    /* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */

    if (ioctl(cPort[comportNumber], TIOCMGET, &status) == -1) {
        tcsetattr(cPort[comportNumber], TCSANOW, oldPortSettings + comportNumber);
        flock(cPort[comportNumber], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "unable to get portstatus";
        return (1);
    }

    status |= TIOCM_DTR; /* turn on DTR */
    status |= TIOCM_RTS; /* turn on RTS */

    if (ioctl(cPort[comportNumber], TIOCMSET, &status) == -1) {
        tcsetattr(cPort[comportNumber], TCSANOW, oldPortSettings + comportNumber);
        flock(cPort[comportNumber], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "unable to set portstatus";
        return (1);
    }

    return (0);
}

int rs232::pollComport(int comportNumber, uint8_t* buf, size_t size) {
    int n = read(cPort[comportNumber], buf, size);
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        }
    }

    return (n);
}

int rs232::sendByte(int comportNumber, const uint8_t byte) {
    int n = write(cPort[comportNumber], &byte, 1);
    if (n < 0) {
        if (errno != EAGAIN) {
            return -1;
        }
    }

    return (0);
}

int rs232::sendBuf(int comportNumber, const uint8_t* buf, size_t size) {
    int n = write(cPort[comportNumber], buf, size);
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        } else {
            return -1;
        }
    }

    return (n);
}

void rs232::closeComport(int comportNumber) {
    int status;

    if (ioctl(cPort[comportNumber], TIOCMGET, &status) == -1) {
        std::cerr << "unable to get portstatus";
    }

    status &= ~TIOCM_DTR; /* turn off DTR */
    status &= ~TIOCM_RTS; /* turn off RTS */

    if (ioctl(cPort[comportNumber], TIOCMSET, &status) == -1) {
        std::cerr << "unable to set portstatus";
    }

    tcsetattr(cPort[comportNumber], TCSANOW, oldPortSettings + comportNumber);
    close(cPort[comportNumber]);

    flock(cPort[comportNumber], LOCK_UN); /* free the port so that others can use it. */
}

/*
Constant  Description
TIOCM_LE        DSR (data set ready/line enable)
TIOCM_DTR       DTR (data terminal ready)
TIOCM_RTS       RTS (request to send)
TIOCM_ST        Secondary TXD (transmit)
TIOCM_SR        Secondary RXD (receive)
TIOCM_CTS       CTS (clear to send)
TIOCM_CAR       DCD (data carrier detect)
TIOCM_CD        see TIOCM_CAR
TIOCM_RNG       RNG (ring)
TIOCM_RI        see TIOCM_RNG
TIOCM_DSR       DSR (data set ready)

http://man7.org/linux/man-pages/man4/tty_ioctl.4.html
 */

bool rs232::isDcdEnabled(int comportNumber) {
    int status;
    ioctl(cPort[comportNumber], TIOCMGET, &status);
    return (status & TIOCM_CAR);
}

bool rs232::isRingEnabled(int comportNumber) {
    int status;
    ioctl(cPort[comportNumber], TIOCMGET, &status);
    return (status & TIOCM_RNG);
}

bool rs232::isCtsEnabled(int comportNumber) {
    int status;
    ioctl(cPort[comportNumber], TIOCMGET, &status);
    return (status & TIOCM_CTS);
}

bool rs232::isDsrEnabled(int comportNumber) {
    int status;
    ioctl(cPort[comportNumber], TIOCMGET, &status);
    return (status & TIOCM_DSR);
}

void rs232::enableDtr(int comportNumber) {
    int status;

    if (ioctl(cPort[comportNumber], TIOCMGET, &status) == -1) {
        std::cerr << "unable to get portstatus";
    }

    status |= TIOCM_DTR; /* turn on DTR */

    if (ioctl(cPort[comportNumber], TIOCMSET, &status) == -1) {
        std::cerr << "unable to set portstatus";
    }
}

void rs232::disableDtr(int comportNumber) {
    int status;

    if (ioctl(cPort[comportNumber], TIOCMGET, &status) == -1) {
        std::cerr << "unable to get portstatus";
    }

    status &= ~TIOCM_DTR; /* turn off DTR */

    if (ioctl(cPort[comportNumber], TIOCMSET, &status) == -1) {
        std::cerr << "unable to set portstatus";
    }
}

void rs232::enableRts(int comportNumber) {
    int status;

    if (ioctl(cPort[comportNumber], TIOCMGET, &status) == -1) {
        std::cerr << "unable to get portstatus";
    }

    status |= TIOCM_RTS; /* turn on RTS */

    if (ioctl(cPort[comportNumber], TIOCMSET, &status) == -1) {
        std::cerr << "unable to set portstatus";
    }
}

void rs232::disableRts(int comportNumber) {
    int status;

    if (ioctl(cPort[comportNumber], TIOCMGET, &status) == -1) {
        std::cerr << "unable to get portstatus";
    }

    status &= ~TIOCM_RTS; /* turn off RTS */

    if (ioctl(cPort[comportNumber], TIOCMSET, &status) == -1) {
        std::cerr << "unable to set portstatus";
    }
}

void rs232::flushRx(int comportNumber) {
    tcflush(cPort[comportNumber], TCIFLUSH);
}

void rs232::flushTx(int comportNumber) {
    tcflush(cPort[comportNumber], TCOFLUSH);
}

void rs232::flushRxTx(int comportNumber) {
    tcflush(cPort[comportNumber], TCIOFLUSH);
}


#else  /* windows */

#include <windows.h>

HANDLE cPort[RS232_PORTNR];
constexpr size_t modeStrLen = 128;
char modeStr[modeStrLen];

int rs232::openComport(int comportNumber, int baudrate, const char* mode, int flowctrl) {
    if ((comportNumber >= RS232_PORTNR) || (comportNumber < 0)) {
        std::cout << "illegal comport number\n";
        return (1);
    }

    switch (baudrate) {
        case 110: strncpy(modeStr, "baud=110", modeStrLen);
            break;
        case 300: strncpy(modeStr, "baud=300", modeStrLen);
            break;
        case 600: strncpy(modeStr, "baud=600", modeStrLen);
            break;
        case 1200: strncpy(modeStr, "baud=1200", modeStrLen);
            break;
        case 2400: strncpy(modeStr, "baud=2400", modeStrLen);
            break;
        case 4800: strncpy(modeStr, "baud=4800", modeStrLen);
            break;
        case 9600: strncpy(modeStr, "baud=9600", modeStrLen);
            break;
        case 19200: strncpy(modeStr, "baud=19200", modeStrLen);
            break;
        case 38400: strncpy(modeStr, "baud=38400", modeStrLen);
            break;
        case 57600: strncpy(modeStr, "baud=57600", modeStrLen);
            break;
        case 115200: strncpy(modeStr, "baud=115200", modeStrLen);
            break;
        case 128000: strncpy(modeStr, "baud=128000", modeStrLen);
            break;
        case 256000: strncpy(modeStr, "baud=256000", modeStrLen);
            break;
        case 500000: strncpy(modeStr, "baud=500000", modeStrLen);
            break;
        case 921600: strncpy(modeStr, "baud=921600", modeStrLen);
            break;
        case 1000000: strncpy(modeStr, "baud=1000000", modeStrLen);
            break;
        case 1500000: strncpy(modeStr, "baud=1500000", modeStrLen);
            break;
        case 2000000: strncpy(modeStr, "baud=2000000", modeStrLen);
            break;
        case 3000000: strncpy(modeStr, "baud=3000000", modeStrLen);
            break;
        default:
            std::cout << "invalid baudrate\n";
            return (1);
    }

    if (strnlen(mode, 4) != 3) {
        std::cout << "invalid mode \"" << mode << "\"\n";
        return (1);
    }

    switch (mode[0]) {
        case '8': strncat(modeStr, " data=8", 8);
            break;
        case '7': strncat(modeStr, " data=7", 8);
            break;
        case '6': strncat(modeStr, " data=6", 8);
            break;
        case '5': strncat(modeStr, " data=5", 8);
            break;
        default:
            std::cout << "invalid number of data-bits '" << static_cast<char> (mode[0]) << "'\n";
            return (1);
    }

    switch (mode[1]) {
        case 'N':
        case 'n': strncat(modeStr, " parity=n", 10);
            break;
        case 'E':
        case 'e': strncat(modeStr, " parity=e", 10);
            break;
        case 'O':
        case 'o': strncat(modeStr, " parity=o", 10);
            break;
        default:
            std::cout << "invalid parity '" << static_cast<char> (mode[1]) << "'\n";
            return (1);
    }

    switch (mode[2]) {
        case '1': strncat(modeStr, " stop=1", 8);
            break;
        case '2': strncat(modeStr, " stop=2", 8);
            break;
        default:
            std::cout << "invalid number of stop bits '" << static_cast<char> (mode[1]) << " '\n";
            return (1);
    }

    if (flowctrl) {
        strncat(modeStr, " xon=off to=off odsr=off dtr=on rts=off", 40);
    } else {
        strncat(modeStr, " xon=off to=off odsr=off dtr=on rts=on", 39);
    }

    /*
    http://msdn.microsoft.com/en-us/library/windows/desktop/aa363145%28v=vs.85%29.aspx
    http://technet.microsoft.com/en-us/library/cc732236.aspx
    https://docs.microsoft.com/en-us/windows/desktop/api/winbase/ns-winbase-_dcb
     */

    cPort[comportNumber] = CreateFileA(COMPORTS[comportNumber],
            GENERIC_READ | GENERIC_WRITE,
            0, /* no share  */
            NULL, /* no security */
            OPEN_EXISTING,
            0, /* no threads */
            NULL); /* no templates */

    if (cPort[comportNumber] == INVALID_HANDLE_VALUE) {
        std::cout << "unable to open comport\n";
        return (1);
    }

    DCB portSettings;
    memset(&portSettings, 0, sizeof (portSettings)); /* clear the new struct  */
    portSettings.DCBlength = sizeof (portSettings);

    if (!BuildCommDCBA(modeStr, &portSettings)) {
        std::cout << "unable to set comport dcb settings\n";
        CloseHandle(cPort[comportNumber]);
        return (1);
    }

    if (flowctrl) {
        portSettings.fOutxCtsFlow = TRUE;
        portSettings.fRtsControl = RTS_CONTROL_HANDSHAKE;
    }

    if (!SetCommState(cPort[comportNumber], &portSettings)) {
        std::cout << "unable to set comport cfg settings\n";
        CloseHandle(cPort[comportNumber]);
        return (1);
    }

    COMMTIMEOUTS Cptimeouts;

    Cptimeouts.ReadIntervalTimeout = MAXDWORD;
    Cptimeouts.ReadTotalTimeoutMultiplier = 0;
    Cptimeouts.ReadTotalTimeoutConstant = 0;
    Cptimeouts.WriteTotalTimeoutMultiplier = 0;
    Cptimeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(cPort[comportNumber], &Cptimeouts)) {
        std::cout << "unable to set comport time-out settings\n";
        CloseHandle(cPort[comportNumber]);
        return (1);
    }

    return (0);
}

int rs232::pollComport(int comportNumber, uint8_t* buf, size_t size) {
    int n;

    /* added the void pointer cast, otherwise gcc will complain about */
    /* "warning: dereferencing type-punned pointer will break strict aliasing rules" */

    ReadFile(cPort[comportNumber], buf, size, (LPDWORD) ((void *) &n), NULL);

    return (n);
}

int rs232::sendByte(int comportNumber, uint8_t byte) {
    int n;

    WriteFile(cPort[comportNumber], &byte, 1, (LPDWORD) ((void *) &n), NULL);

    if (n < 0) {
        return (-1);
    }

    return (0);
}

int rs232::sendBuf(int comportNumber, const unsigned* buf, int size) {
    int n;

    if (WriteFile(cPort[comportNumber], buf, size, (LPDWORD) ((void *) &n), NULL)) {
        return (n);
    }

    return (-1);
}

void rs232::closeComport(int comportNumber) {
    CloseHandle(cPort[comportNumber]);
}

/*
http://msdn.microsoft.com/en-us/library/windows/desktop/aa363258%28v=vs.85%29.aspx
 */

bool rs232::isDcdEnabled(int comportNumber) {
    int status;
    GetCommModemStatus(cPort[comportNumber], (LPDWORD) ((void *) &status));
    return (status & MS_RLSD_ON);
}

bool rs232::isRingEnabled(int comportNumber) {
    int status;
    GetCommModemStatus(cPort[comportNumber], (LPDWORD) ((void *) &status));
    return (status & MS_RING_ON);
}

bool rs232::isCtsEnabled(int comportNumber) {
    int status;
    GetCommModemStatus(cPort[comportNumber], (LPDWORD) ((void *) &status));
    return (status & MS_CTS_ON);
}

bool rs232::isDsrEnabled(int comportNumber) {
    int status;
    GetCommModemStatus(cPort[comportNumber], (LPDWORD) ((void *) &status));
    return (status & MS_DSR_ON);
}

void rs232::enableDtr(int comportNumber) {
    EscapeCommFunction(cPort[comportNumber], SETDTR);
}

void rs232::disableDtr(int comportNumber) {
    EscapeCommFunction(cPort[comportNumber], CLRDTR);
}

void rs232::enableRts(int comportNumber) {
    EscapeCommFunction(cPort[comportNumber], SETRTS);
}

void rs232::disableRts(int comportNumber) {
    EscapeCommFunction(cPort[comportNumber], CLRRTS);
}

/*
https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428%28v=vs.85%29.aspx
 */

void rs232::flushRx(int comportNumber) {
    PurgeComm(cPort[comportNumber], PURGE_RXCLEAR | PURGE_RXABORT);
}

void rs232::flushTx(int comportNumber) {
    PurgeComm(cPort[comportNumber], PURGE_TXCLEAR | PURGE_TXABORT);
}

void rs232::flushRxTx(int comportNumber) {
    PurgeComm(cPort[comportNumber], PURGE_RXCLEAR | PURGE_RXABORT);
    PurgeComm(cPort[comportNumber], PURGE_TXCLEAR | PURGE_TXABORT);
}


#endif

void rs232::cputs(int comportNumber, const char* text) /* sends a string to serial port */ {
    while (*text != 0) sendByte(comportNumber, *(text++));
}

/* return index in comports matching to device name or -1 if not found */
int rs232::getPortNr(const char* devname) {

    constexpr size_t strLen = 32;
    char str[strLen];

#if defined(__linux__) || defined(__FreeBSD__)   /* Linux & FreeBSD */
    strncpy(str, "/dev/", strLen);
#else  /* windows */
    strncpy(str, "\\\\.\\", strLen);
#endif
    strncat(str, devname, strLen);
    str[strLen - 1] = '\0';

    for (int i = 0; i < RS232_PORTNR; i++) {
        if (!strncmp(COMPORTS[i], str, strLen)) {
            return i;
        }
    }

    return -1; /* device not found */
}
