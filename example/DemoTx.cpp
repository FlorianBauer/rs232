
/**************************************************

file: DemoTx.cpp
purpose: simple demo that transmits characters to
the serial port and print them on the screen,
exit the program by pressing Ctrl-C

compile with the command: g++ -std=c++11 DemoTx.cpp Rs232.cpp -Wall -Wextra -O2 -o test_tx

 **************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "Rs232.h"

int main() {
    int i = 0;
    int cPortNr = 0; /* /dev/ttyS0 (COM1 on windows) */
    int bdrate = 9600; /* 9600 baud */
    char mode[] = {'8', 'N', '1', 0};
    constexpr size_t strLen = 512;
    char str[2][strLen];

    strncpy(str[0], "The quick brown fox jumped over the lazy grey dog.\n", strLen);
    strncpy(str[1], "Happy serial programming!\n", strLen);

    if (rs232::openComport(cPortNr, bdrate, mode, 0)) {
        printf("Can not open comport\n");
        return (0);
    }

    while (1) {
        rs232::cputs(cPortNr, str[i]);
        printf("sent: %s\n", str[i]);

#ifdef _WIN32
        Sleep(1000);
#else
        usleep(1000000); /* sleep for 1 Second */
#endif

        i++;
        i %= 2;
    }

    return (0);
}
