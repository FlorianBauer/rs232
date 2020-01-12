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
#include "Rs232.h"

#include <iostream>
#include <cstring>

using namespace rs232;

#if defined(__linux__) || defined(__FreeBSD__)   /* Linux & FreeBSD */

#include <algorithm>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>

// Enable only if used with a virtual connection via `socat` and `PTY`.
// #define VIRTUAL_CONNECTION

/**
 * Use a struct template to statically initialize the comports with invalid file handles (-1).
 */
template<size_t N>
struct Com {
    int port[N];

    Com() {
        std::fill_n(port, N, -1);
    }
};

static struct Com<MAX_COMPORTS> com;
static struct termios oldPortSettings[MAX_COMPORTS];
struct termios newPortSettings;

int rs232::openComport(unsigned portIdx,
        int baudrate,
        const std::string& mode,
        bool enableFlowCtrl) {

    if (portIdx >= MAX_COMPORTS) {
        std::cout << "Illegal comport index.\n";
        return -1;
    }

    int baudr;
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
            std::cout << "Invalid baudrate.\n";
            return -2;
    }

    int cbits = CS8;
    int cpar = 0;
    int ipar = IGNPAR;
    int bstop = 0;

    if (mode.length() != 3) {
        std::cout << "Invalid mode \"" << mode << "\".\n";
        return -3;
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
            std::cout << "Invalid number of data-bits '" << static_cast<char> (mode[0]) << "'.\n";
            return -4;
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
            std::cout << "Invalid parity '" << static_cast<char> (mode[1]) << "'.\n";
            return -5;
    }

    switch (mode[2]) {
        case '1': bstop = 0;
            break;
        case '2': bstop = CSTOPB;
            break;
        default:
            std::cout << "Invalid number of stop bits '" << static_cast<char> (mode[2]) << "'.\n";
            return -6;
    }

    /*
    http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html
    http://man7.org/linux/man-pages/man3/termios.3.html
     */

    com.port[portIdx] = open(COMPORTS[portIdx], O_RDWR | O_NOCTTY | O_NDELAY);
    if (com.port[portIdx] == -1) {
        std::cerr << "Unable to open comport.\n";
        return -7;
    }

    /* lock access so that another process can't also use the port */
    if (flock(com.port[portIdx], LOCK_EX | LOCK_NB) != 0) {
        close(com.port[portIdx]);
        std::cerr << "Another process has locked the comport.\n";
        return -8;
    }

    int error = tcgetattr(com.port[portIdx], oldPortSettings + portIdx);
    if (error == -1) {
        close(com.port[portIdx]);
        flock(com.port[portIdx], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "Unable to read port settings.\n";
        return -9;
    }
    memset(&newPortSettings, 0, sizeof (newPortSettings)); /* clear the new struct */

    newPortSettings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
    if (enableFlowCtrl) {
        newPortSettings.c_cflag |= CRTSCTS;
    }
    newPortSettings.c_iflag = ipar;
    newPortSettings.c_oflag = 0;
    newPortSettings.c_lflag = 0;
    newPortSettings.c_cc[VMIN] = 0; /* block untill n bytes are received */
    newPortSettings.c_cc[VTIME] = 0; /* block untill a timer expires (n * 100 mSec.) */

    cfsetispeed(&newPortSettings, baudr);
    cfsetospeed(&newPortSettings, baudr);

    error = tcsetattr(com.port[portIdx], TCSANOW, &newPortSettings);
    if (error == -1) {
        tcsetattr(com.port[portIdx], TCSANOW, oldPortSettings + portIdx);
        close(com.port[portIdx]);
        flock(com.port[portIdx], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "Unable to adjust port settings.\n";
        return -10;
    }

    /* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */
#ifndef VIRTUAL_CONNECTION
    int status;
    if (ioctl(com.port[portIdx], TIOCMGET, &status) == -1) {
        tcsetattr(com.port[portIdx], TCSANOW, oldPortSettings + portIdx);
        flock(com.port[portIdx], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "Unable to get port status.\n";
        return -11;
    }

    status |= TIOCM_DTR; /* turn on DTR */
    status |= TIOCM_RTS; /* turn on RTS */

    if (ioctl(com.port[portIdx], TIOCMSET, &status) == -1) {
        tcsetattr(com.port[portIdx], TCSANOW, oldPortSettings + portIdx);
        flock(com.port[portIdx], LOCK_UN); /* free the port so that others can use it. */
        std::cerr << "Unable to set port status.\n";
        return -12;
    }
#endif

    return 0;
}

int rs232::pollComport(unsigned portIdx, uint8_t* buf, size_t size) {
    int n = read(com.port[portIdx], buf, size);
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        }
    }

    return n;
}

int rs232::sendByte(unsigned portIdx, const uint8_t byte) {
    int n = write(com.port[portIdx], &byte, 1);
    if (n < 0) {
        if (errno != EAGAIN) {
            return -1;
        }
    }

    return 0;
}

int rs232::sendBuf(unsigned portIdx, const uint8_t* buf, size_t size) {
    int n = write(com.port[portIdx], buf, size);
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        } else {
            return -1;
        }
    }

    return n;
}

int rs232::sendBuf(unsigned portIdx, const char* buf, size_t size) {
    int n = write(com.port[portIdx], buf, size);
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        } else {
            return -1;
        }
    }
    return n;
}

int rs232::sendBuf(unsigned portIdx, const std::vector<uint8_t>& buf) {
    int n = write(com.port[portIdx], buf.data(), buf.size());
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        } else {
            return -1;
        }
    }
    return n;
}

int rs232::sendBuf(unsigned portIdx, const std::string& buf) {
    int n = write(com.port[portIdx], buf.c_str(), buf.length());
    if (n < 0) {
        if (errno == EAGAIN) {
            return 0;
        } else {
            return -1;
        }
    }
    return n;
}

void rs232::closeComport(unsigned portIdx) {
#ifndef VIRTUAL_CONNECTION
    int status;
    if (ioctl(com.port[portIdx], TIOCMGET, &status) == -1) {
        std::cerr << "Unable to get port status.\n";
    }

    status &= ~TIOCM_DTR; /* turn off DTR */
    status &= ~TIOCM_RTS; /* turn off RTS */

    if (ioctl(com.port[portIdx], TIOCMSET, &status) == -1) {
        std::cerr << "Unable to set port status.\n";
    }
#endif
    tcsetattr(com.port[portIdx], TCSANOW, oldPortSettings + portIdx);
    close(com.port[portIdx]);
    flock(com.port[portIdx], LOCK_UN); /* free the port so that others can use it. */
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

bool rs232::isDcdEnabled(unsigned portIdx) {
    int status = 0;
#ifndef VIRTUAL_CONNECTION
    ioctl(com.port[portIdx], TIOCMGET, &status);
#endif
    return (status & TIOCM_CAR);
}

bool rs232::isRingEnabled(unsigned portIdx) {
    int status = 0;
#ifndef VIRTUAL_CONNECTION
    ioctl(com.port[portIdx], TIOCMGET, &status);
#endif
    return (status & TIOCM_RNG);
}

bool rs232::isCtsEnabled(unsigned portIdx) {
    int status = 0;
#ifndef VIRTUAL_CONNECTION
    ioctl(com.port[portIdx], TIOCMGET, &status);
#endif
    return (status & TIOCM_CTS);
}

bool rs232::isDsrEnabled(unsigned portIdx) {
    int status = 0;
#ifndef VIRTUAL_CONNECTION
    ioctl(com.port[portIdx], TIOCMGET, &status);
#endif
    return (status & TIOCM_DSR);
}

void rs232::enableDtr(unsigned portIdx) {
#ifndef VIRTUAL_CONNECTION
    int status;
    if (ioctl(com.port[portIdx], TIOCMGET, &status) == -1) {
        std::cerr << "Unable to get port status.\n";
    }
    status |= TIOCM_DTR; /* turn on DTR */
    if (ioctl(com.port[portIdx], TIOCMSET, &status) == -1) {
        std::cerr << "Unable to set port status.\n";
    }
#endif
}

void rs232::disableDtr(unsigned portIdx) {
#ifndef VIRTUAL_CONNECTION
    int status;
    if (ioctl(com.port[portIdx], TIOCMGET, &status) == -1) {
        std::cerr << "Unable to get port status.\n";
    }
    status &= ~TIOCM_DTR; /* turn off DTR */
    if (ioctl(com.port[portIdx], TIOCMSET, &status) == -1) {
        std::cerr << "Unable to set port status.\n";
    }
#endif
}

void rs232::enableRts(unsigned portIdx) {
#ifndef VIRTUAL_CONNECTION
    int status;
    if (ioctl(com.port[portIdx], TIOCMGET, &status) == -1) {
        std::cerr << "Unable to get port status.\n";
    }
    status |= TIOCM_RTS; /* turn on RTS */
    if (ioctl(com.port[portIdx], TIOCMSET, &status) == -1) {
        std::cerr << "Unable to set port status.\n";
    }
#endif
}

void rs232::disableRts(unsigned portIdx) {
#ifndef VIRTUAL_CONNECTION
    int status;
    if (ioctl(com.port[portIdx], TIOCMGET, &status) == -1) {
        std::cerr << "Unable to get port status.\n";
    }
    status &= ~TIOCM_RTS; /* turn off RTS */
    if (ioctl(com.port[portIdx], TIOCMSET, &status) == -1) {
        std::cerr << "Unable to set port status.\n";
    }
#endif
}

void rs232::flushRx(unsigned portIdx) {
    tcflush(com.port[portIdx], TCIFLUSH);
}

void rs232::flushTx(unsigned portIdx) {
    tcflush(com.port[portIdx], TCOFLUSH);
}

void rs232::flushRxTx(unsigned portIdx) {
    tcflush(com.port[portIdx], TCIOFLUSH);
}


#else  /* windows */

#include <windows.h>

static HANDLE comPort[MAX_COMPORTS];
constexpr size_t modeStrLen = 128;
char modeStr[modeStrLen];

int rs232::openComport(unsigned portIdx,
        int baudrate,
        const std::string& mode,
        bool enableFlowCtrl) {

    if (portIdx >= MAX_COMPORTS) {
        std::cout << "Illegal comport number.\n";
        return -1;
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
            std::cout << "Invalid baudrate.\n";
            return -2;
    }

    if (mode.length() != 3) {
        std::cout << "Invalid mode \"" << mode << "\".\n";
        return -3;
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
            std::cout << "Invalid number of data-bits '" << static_cast<char> (mode[0]) << "'.\n";
            return -4;
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
            std::cout << "Invalid parity '" << static_cast<char> (mode[1]) << "'.\n";
            return -5;
    }

    switch (mode[2]) {
        case '1': strncat(modeStr, " stop=1", 8);
            break;
        case '2': strncat(modeStr, " stop=2", 8);
            break;
        default:
            std::cout << "Invalid number of stop bits '" << static_cast<char> (mode[1]) << "'.\n";
            return -6;
    }

    if (enableFlowCtrl) {
        strncat(modeStr, " xon=off to=off odsr=off dtr=on rts=off", 40);
    } else {
        strncat(modeStr, " xon=off to=off odsr=off dtr=on rts=on", 39);
    }

    /*
    http://msdn.microsoft.com/en-us/library/windows/desktop/aa363145%28v=vs.85%29.aspx
    http://technet.microsoft.com/en-us/library/cc732236.aspx
    https://docs.microsoft.com/en-us/windows/desktop/api/winbase/ns-winbase-_dcb
     */

    comPort[portIdx] = CreateFileA(COMPORTS[portIdx],
            GENERIC_READ | GENERIC_WRITE,
            0, /* no share  */
            NULL, /* no security */
            OPEN_EXISTING,
            0, /* no threads */
            NULL); /* no templates */

    if (comPort[portIdx] == INVALID_HANDLE_VALUE) {
        std::cout << "Unable to open comport.\n";
        return -7;
    }

    DCB portSettings;
    memset(&portSettings, 0, sizeof (portSettings)); /* clear the new struct  */
    portSettings.DCBlength = sizeof (portSettings);

    if (!BuildCommDCBA(modeStr, &portSettings)) {
        std::cout << "Unable to set comport dcb settings.\n";
        CloseHandle(comPort[portIdx]);
        return -8;
    }

    if (enableFlowCtrl) {
        portSettings.fOutxCtsFlow = TRUE;
        portSettings.fRtsControl = RTS_CONTROL_HANDSHAKE;
    }

    if (!SetCommState(comPort[portIdx], &portSettings)) {
        std::cout << "Unable to set comport cfg settings.\n";
        CloseHandle(comPort[portIdx]);
        return -9;
    }

    COMMTIMEOUTS Cptimeouts;
    Cptimeouts.ReadIntervalTimeout = MAXDWORD;
    Cptimeouts.ReadTotalTimeoutMultiplier = 0;
    Cptimeouts.ReadTotalTimeoutConstant = 0;
    Cptimeouts.WriteTotalTimeoutMultiplier = 0;
    Cptimeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(comPort[portIdx], &Cptimeouts)) {
        std::cout << "Unable to set comport time-out settings.\n";
        CloseHandle(comPort[portIdx]);
        return -10;
    }

    return 0;
}

int rs232::pollComport(unsigned portIdx, uint8_t* buf, size_t size) {
    int n;
    /* added the void pointer cast, otherwise gcc will complain about */
    /* "warning: dereferencing type-punned pointer will break strict aliasing rules" */
    ReadFile(comPort[portIdx], buf, size, (LPDWORD) ((void *) &n), NULL);
    return n;
}

int rs232::sendByte(unsigned portIdx, uint8_t byte) {
    int n;
    WriteFile(comPort[portIdx], &byte, 1, (LPDWORD) ((void *) &n), NULL);
    if (n < 0) {
        return -1;
    }
    return 0;
}

int rs232::sendBuf(unsigned portIdx, const uint8_t* buf, size_t size) {
    int n;
    if (WriteFile(comPort[portIdx], buf, size, (LPDWORD) ((void *) &n), NULL)) {
        return n;
    }
    return -1;
}

int rs232::sendBuf(unsigned portIdx, const char* buf, size_t size) {
    int n;
    if (WriteFile(comPort[portIdx], buf, size, (LPDWORD) ((void *) &n), NULL)) {
        return n;
    }
    return -1;
}

int rs232::sendBuf(unsigned portIdx, const std::vector<uint8_t>& buf) {
    int n;
    if (WriteFile(comPort[portIdx], buf.data(), buf.size(), (LPDWORD) ((void *) &n), NULL)) {
        return n;
    }
    return -1;
}

int rs232::sendBuf(unsigned portIdx, const std::string& buf) {
    int n;
    if (WriteFile(comPort[portIdx], buf.c_str(), buf.length(), (LPDWORD) ((void *) &n), NULL)) {
        return n;
    }
    return -1;
}

void rs232::closeComport(unsigned portIdx) {
    CloseHandle(comPort[portIdx]);
}

/*
http://msdn.microsoft.com/en-us/library/windows/desktop/aa363258%28v=vs.85%29.aspx
 */

bool rs232::isDcdEnabled(unsigned portIdx) {
    int status;
    GetCommModemStatus(comPort[portIdx], (LPDWORD) ((void *) &status));
    return (status & MS_RLSD_ON);
}

bool rs232::isRingEnabled(unsigned portIdx) {
    int status;
    GetCommModemStatus(comPort[portIdx], (LPDWORD) ((void *) &status));
    return (status & MS_RING_ON);
}

bool rs232::isCtsEnabled(unsigned portIdx) {
    int status;
    GetCommModemStatus(comPort[portIdx], (LPDWORD) ((void *) &status));
    return (status & MS_CTS_ON);
}

bool rs232::isDsrEnabled(unsigned portIdx) {
    int status;
    GetCommModemStatus(comPort[portIdx], (LPDWORD) ((void *) &status));
    return (status & MS_DSR_ON);
}

void rs232::enableDtr(unsigned portIdx) {
    EscapeCommFunction(comPort[portIdx], SETDTR);
}

void rs232::disableDtr(unsigned portIdx) {
    EscapeCommFunction(comPort[portIdx], CLRDTR);
}

void rs232::enableRts(unsigned portIdx) {
    EscapeCommFunction(comPort[portIdx], SETRTS);
}

void rs232::disableRts(unsigned portIdx) {
    EscapeCommFunction(comPort[portIdx], CLRRTS);
}

/*
https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428%28v=vs.85%29.aspx
 */

void rs232::flushRx(unsigned portIdx) {
    PurgeComm(comPort[portIdx], PURGE_RXCLEAR | PURGE_RXABORT);
}

void rs232::flushTx(unsigned portIdx) {
    PurgeComm(comPort[portIdx], PURGE_TXCLEAR | PURGE_TXABORT);
}

void rs232::flushRxTx(unsigned portIdx) {
    PurgeComm(comPort[portIdx], PURGE_RXCLEAR | PURGE_RXABORT);
    PurgeComm(comPort[portIdx], PURGE_TXCLEAR | PURGE_TXABORT);
}


#endif

/* sends a string to serial port */
void rs232::cputs(unsigned portIdx, const char* text) {
    while (*text != 0) {
        sendByte(portIdx, *(text++));
    }
}

/* return index in comports matching to device name or -1 if not found */
int rs232::getPortIdx(const std::string& devName) {
    if (!devName.empty() && devName.length() < 24) {
        std::string absDevName;
        if (devName.compare(0, strlen(DEV_PATH), DEV_PATH) == 0) {
            absDevName = devName;
        } else {
            absDevName = DEV_PATH + devName;
        }

        for (unsigned i = 0; i < MAX_COMPORTS; i++) {
            if (absDevName.compare(COMPORTS[i]) == 0) {
                return i;
            }
        }
    }
    return -1;
}
