#
#
# Author: Teunis van Beelen
#
# email: teuniz@protonmail.com
#
#

CC = g++
CFLAGS = -std=c++11 -Wall -Wextra -Wshadow -Wformat-nonliteral -Wformat-security -Wtype-limits -O2
SRC_DIR = -I../

objects = Rs232.o

all: test_rx test_tx

test_rx : $(objects) DemoRx.o
	$(CC) $(objects) $(SRC_DIR) DemoRx.o -o test_rx

test_tx : $(objects) DemoTx.o
	$(CC) $(objects) $(SRC_DIR) DemoTx.o -o test_tx

DemoRx.o : DemoRx.cpp ../Rs232.h
	$(CC) $(CFLAGS) $(SRC_DIR) -c DemoRx.cpp -o DemoRx.o

DemoTx.o : DemoTx.cpp ../Rs232.h
	$(CC) $(CFLAGS) $(SRC_DIR) -c DemoTx.cpp -o DemoTx.o

Rs232.o : ../Rs232.h ../Rs232.cpp
	$(CC) $(CFLAGS) -c $(SRC_DIR) ../Rs232.cpp -o Rs232.o

clean :
	$(RM) test_rx test_tx $(objects) DemoRx.o DemoTx.o Rs232.o

