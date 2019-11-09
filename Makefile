#
#
# Author: Teunis van Beelen
#
# email: teuniz@protonmail.com
#
#

CC = g++
CFLAGS = -std=c++11 -Wall -Wextra -Wshadow -Wformat-nonliteral -Wformat-security -Wtype-limits -O2

objects = Rs232.o

all: test_rx test_tx

test_rx : $(objects) DemoRx.o
	$(CC) $(objects) DemoRx.o -o test_rx

test_tx : $(objects) DemoTx.o
	$(CC) $(objects) DemoTx.o -o test_tx

DemoRx.o : DemoRx.cpp Rs232.h
	$(CC) $(CFLAGS) -c DemoRx.cpp -o DemoRx.o

DemoTx.o : DemoTx.cpp Rs232.h
	$(CC) $(CFLAGS) -c DemoTx.cpp -o DemoTx.o

Rs232.o : Rs232.h Rs232.cpp
	$(CC) $(CFLAGS) -c Rs232.cpp -o Rs232.o

clean :
	$(RM) test_rx test_tx $(objects) DemoRx.o DemoTx.o Rs232.o
