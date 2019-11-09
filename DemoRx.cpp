
/**************************************************

file: DemoRx.cpp
purpose: simple demo that receives characters from
the serial port and print them on the screen,
exit the program by pressing Ctrl-C

compile with the command: g++ -std=c++11 DemoRx.cpp Rs232.cpp -Wall -Wextra -O2 -o test_rx

 **************************************************/

#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "Rs232.h"

int main() {
    int i;
    int n;
    int cPortNr = 0; /* /dev/ttyS0 (COM1 on windows) */
    int bdrate = 9600; /* 9600 baud */
    constexpr size_t bufLen = 4096;
    unsigned char buf[bufLen];

    char mode[] = {'8', 'N', '1', 0};


    if (rs232::openComport(cPortNr, bdrate, mode, 0)) {
        printf("Can not open comport\n");

        return (0);
    }

    while (1) {
        n = rs232::pollComport(cPortNr, buf, bufLen - 1);

        if (n > 0) {
            buf[n] = 0; /* always put a "null" at the end of a string! */

            for (i = 0; i < n; i++) {
                if (buf[i] < 32) /* replace unreadable control-codes by dots */ {
                    buf[i] = '.';
                }
            }

            printf("received %i bytes: %s\n", n, (char *) buf);
        }

#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000); /* sleep for 100 milliSeconds */
#endif
    }

    return (0);
}
