# RS-232
RS-232 for Linux, FreeBSD and Windows

Website: https://www.teuniz.net/RS-232/


You don't need to call `rs232::pollComport()` when you only want to send characters. Sending and 
receiving do not influence each other.

The OS (kernel) has an internal buffer of 4096 bytes (for traditional onboard serial ports).
USB/Serial-converter drivers use much bigger buffers (multiples of 4096). If this buffer is full 
and a new character arrives on the serial port, the oldest character in the buffer will be 
overwritten and thus will be lost.

After a successful call to `rs232::openComport()`, the OS will start to buffer incoming 
characters.

Tip: To get access to the serial port on Linux, you need to be a member of the group "dialout".

Note: Traditional (on-board) UART's usually have a speed limit of max. 115200 baud.
      Special cards and USB to Serial converters can usually be set to higher baudrates.

Idx | Linux    | Windows
----|----------|--------
0   | ttyS0    | COM1
1   | ttyS1    | COM2
2   | ttyS2    | COM3
3   | ttyS3    | COM4
4   | ttyS4    | COM5
5   | ttyS5    | COM6
6   | ttyS6    | COM7
7   | ttyS7    | COM8
8   | ttyS8    | COM9
9   | ttyS9    | COM10
10  | ttyS10   | COM11
11  | ttyS11   | COM12
12  | ttyS12   | COM13
13  | ttyS13   | COM14
14  | ttyS14   | COM15
15  | ttyS15   | COM16
16  | ttyUSB0  | COM17
17  | ttyUSB1  | COM18
18  | ttyUSB2  | COM19
19  | ttyUSB3  | COM20
20  | ttyUSB4  | COM21
21  | ttyUSB5  | COM22
22  | ttyAMA0  | COM23
23  | ttyAMA1  | COM24
24  | ttyACM0  | COM25
25  | ttyACM1  | COM26
26  | rfcomm0  | COM27
27  | rfcomm1  | COM28
28  | ircomm0  | COM29
29  | ircomm1  | COM30
30  | cuau0    | COM31
31  | cuau1    | COM32
32  | cuau2    | n.a.
33  | cuau3    | n.a.
34  | cuaU0    | n.a.
35  | cuaU1    | n.a.
36  | cuaU2    | n.a.
37  | cuaU3    | n.a.

Linux    | Windows
---------|--------
50       | n.a.
75       | n.a.
110      | 110
134      | n.a.
150      | n.a.
200      | n.a.
300      | 300
600      | 600
1200     | 1200
1800     | n.a.
2400     | 2400
4800     | 4800
9600     | 9600
19200    | 19200
38400    | 38400
57600    | 57600
115200   | 115200
230400   | 128000
460800   | 256000
500000   | 500000
576000   | n.a.
921600   | 921600
1000000  | 1000000
1152000  | n.a.
1500000  | 1500000
2000000  | 2000000
2500000  | n.a.
3000000  | 3000000
3500000  | n.a.
4000000  | n.a.

Mode
```
8N1
8O1
8E1
8N2
8O2
8E2
7N1
7O1
7E1
7N2
7O2
7E2
6N1
6O1
6E1
6N2
6O2
6E2
5N1
5O1
5E1
5N2
5O2
5E2
```