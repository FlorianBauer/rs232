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
#include <string>
#include <vector>

namespace rs232 {

    /**
     * List of available comports.
     * 
     *  Idx | Linux    | Windows
     * -----|----------|---------
     *   0  | ttyS0    | COM1
     *   1  | ttyS1    | COM2
     *   2  | ttyS2    | COM3
     *   3  | ttyS3    | COM4
     *   4  | ttyS4    | COM5
     *   5  | ttyS5    | COM6
     *   6  | ttyS6    | COM7
     *   7  | ttyS7    | COM8
     *   8  | ttyS8    | COM9
     *   9  | ttyS9    | COM10
     *  10  | ttyS10   | COM11
     *  11  | ttyS11   | COM12
     *  12  | ttyS12   | COM13
     *  13  | ttyS13   | COM14
     *  14  | ttyS14   | COM15
     *  15  | ttyS15   | COM16
     *  16  | ttyUSB0  | COM17
     *  17  | ttyUSB1  | COM18
     *  18  | ttyUSB2  | COM19
     *  19  | ttyUSB3  | COM20
     *  20  | ttyUSB4  | COM21
     *  21  | ttyUSB5  | COM22
     *  22  | ttyAMA0  | COM23
     *  23  | ttyAMA1  | COM24
     *  24  | ttyACM0  | COM25
     *  25  | ttyACM1  | COM26
     *  26  | rfcomm0  | COM27
     *  27  | rfcomm1  | COM28
     *  28  | ircomm0  | COM29
     *  29  | ircomm1  | COM30
     *  30  | cuau0    | COM31
     *  31  | cuau1    | COM32
     *  32  | cuau2    | n.a.
     *  33  | cuau3    | n.a.
     *  34  | cuaU0    | n.a.
     *  35  | cuaU1    | n.a.
     *  36  | cuaU2    | n.a.
     *  37  | cuaU3    | n.a.
     */

#if defined(__linux__) || defined(__FreeBSD__)

    static constexpr const char* const COMPORTS[] = {
        /*  0 */ "/dev/ttyS0",
        /*  1 */ "/dev/ttyS1",
        /*  2 */ "/dev/ttyS2",
        /*  3 */ "/dev/ttyS3",
        /*  4 */ "/dev/ttyS4",
        /*  5 */ "/dev/ttyS5",
        /*  6 */ "/dev/ttyS6",
        /*  7 */ "/dev/ttyS7",
        /*  8 */ "/dev/ttyS8",
        /*  9 */ "/dev/ttyS9",
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

    /// Path to the device ports. Is OS dependent e.g. Linux = `"/dev/"`, Windows = `"\\.\"`.
    static constexpr const char DEV_PATH[] = "/dev/";

#else // windows

    static constexpr const char* const COMPORTS[] = {
        /*  0 */ "\\\\.\\COM1",
        /*  1 */ "\\\\.\\COM2",
        /*  2 */ "\\\\.\\COM3",
        /*  3 */ "\\\\.\\COM4",
        /*  4 */ "\\\\.\\COM5",
        /*  5 */ "\\\\.\\COM6",
        /*  6 */ "\\\\.\\COM7",
        /*  7 */ "\\\\.\\COM8",
        /*  8 */ "\\\\.\\COM9",
        /*  9 */ "\\\\.\\COM10",
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

    /// Path to the device ports. Is OS dependent e.g. Linux = `"/dev/"`, Windows = `"\\.\"`.
    static constexpr const char DEV_PATH[] = "\\\\.\\";

#endif

    /// The max. number of comports.
    static constexpr unsigned MAX_COMPORTS = sizeof(COMPORTS) / sizeof(COMPORTS[0]);

    /**
     * Opens the comport. In case the comport is already opened (by another process), it will not 
     * open the port but raise an error instead.
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     * @param baudrate Expressed in baud per second i.e 115200 (see the list of possible baudrates).
     * @param mode Is a string in the form of "8N1", "7E2", etc. 8N1 means eight databits, no 
     *             parity, one stopbit. If in doubt, use 8N1 (see the list of possible modes).
     * @param enableFlowCtrl If true, hardware flow control is enabled using the RTS/CTS lines.
     * @return 0 on success, otherwise a negative value.
     */
    int openComport(unsigned portIdx,
            int baudrate,
            const std::string& mode,
            bool enableFlowCtrl=false);

    /**
     * Gets bytes (characters) from the serial port (if any). It does not block or wait, it returns 
     * immediately, no matter if any characters have been received or not. After successfully 
     * opening the COM-port, connect this function to a timer. The timer should have an interval of 
     * approx. 20 to 100 Milliseconds. Do not forget to stop the timer before closing the COM-port.
     * Always check the return value! The return value tells you how many bytes are actually 
     * received and present in your buffer.
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     * @param[out] buf Pointer to the receiving buffer.
     * @param size The size of the buffer in bytes.
     * @return The amount of received bytes (characters) into the buffer. This can be less than 
     *         size or zero.
     */
    int pollComport(unsigned portIdx, uint8_t* buf, size_t size);

    /**
     * Sends a byte via the serial port.
     * 
     * @param portIdx The index of the comport. Starts with 0 (see list of #COMPORTS).
     * @param byte The byte to send.
     * @return 0 on success, otherwise a negative value.
     */
    int sendByte(unsigned portIdx, const uint8_t byte);

    /**
     * Sends multiple bytes via the serial port. This function blocks (it returns after all the
     * bytes have been processed).
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     * @param[in] buf Pointer to the buffer.
     * @param size The size of the buffer in bytes.
     * @return The amount of bytes sent or a negative value on error.
     */
    int sendBuf(unsigned portIdx, const uint8_t* buf, size_t size);

    /**
     * Sends multiple bytes via the serial port. This function blocks (it returns after all the
     * bytes have been processed).
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     * @param[in] buf Pointer to the buffer.
     * @param size The size of the buffer in bytes.
     * @return The amount of bytes sent or a negative value on error.
     */
    int sendBuf(unsigned portIdx, const char* buf, size_t size);

    /**
     * Sends multiple bytes via the serial port. This function blocks (it returns after all the
     * bytes have been processed).
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     * @param[in] buf The data to send.
     * @return The amount of bytes sent or a negative value on error.
     */
    int sendBuf(unsigned portIdx, const std::vector<uint8_t>& buf);

    /**
     * Sends multiple bytes via the serial port. This function blocks (it returns after all the
     * bytes have been processed).
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     * @param[in] buf The data to send.
     * @return The amount of bytes sent or a negative value on error.
     */
    int sendBuf(unsigned portIdx, const std::string& buf);

    /**
     * Closes the serial port.
     * 
     * @param portIdx The index of the comport (see #COMPORTS).
     */
    void closeComport(unsigned portIdx);

    /**
     * Sends a string via the serial port. String must be null-terminated.
     * 
     * @param portIdx The index of the comport.
     * @param text The string to send.
     */
    void cputs(unsigned portIdx, const char* text);

    /**
     * Checks the status of the DCD (data carrier detect)-pin.
     * 
     * @param portIdx The index of the comport.
     * @return true when the the DCD line is high (active state), otherwise false.
     */
    bool isDcdEnabled(unsigned portIdx);

    /**
     * Checks the status of the RING-pin.
     * 
     * @param portIdx The index of the comport.
     * @return true when the the RING line is high (active state), otherwise false.
     */
    bool isRingEnabled(unsigned portIdx);

    /**
     * Checks the status of the CTS (clear to send)-pin.
     * 
     * @param portIdx The index of the comport.
     * @return true when the the CTS line is high (active state), otherwise false.
     */
    bool isCtsEnabled(unsigned portIdx);

    /**
     * Checks the status of the DSR (data set ready)-pin.
     * 
     * @param portIdx The index of the comport.
     * @return true when the the DSR line is high (active state), otherwise false.
     */
    bool isDsrEnabled(unsigned portIdx);

    /**
     * Sets the DTR (data terminal ready) line high (active state).
     * 
     * @param portIdx The index of the comport.
     */
    void enableDtr(unsigned portIdx);

    /**
     * Sets the DTR (data terminal ready) line low (non active state).
     * 
     * @param portIdx The index of the comport.
     */
    void disableDtr(unsigned portIdx);

    /**
     * Sets the RTS (request to send) line high (active state).
     * Do not use this function if hardware flow control is enabled!
     * 
     * @param portIdx The index of the comport.
     */
    void enableRts(unsigned portIdx);

    /**
     * Sets the RTS (request to send) line low (non active state).
     * 
     * @param portIdx The index of the comport.
     */
    void disableRts(unsigned portIdx);

    /**
     * Flushes data received but not read.
     * 
     * @param portIdx The index of the comport.
     */
    void flushRx(unsigned portIdx);

    /**
     * Flushes data written but not transmitted.
     * 
     * @param portIdx The index of the comport.
     */
    void flushTx(unsigned portIdx);

    /**
     * Flushes both data received but not read, and data written but not transmitted.
     * 
     * @param portIdx The index of the comport.
     */
    void flushRxTx(unsigned portIdx);

    /**
     * Returns the comport number based on the device name. 
     *  This function does not check if device actually exists! Examples for valid device names: 
     * "ttyS0", "/dev/ttyS1", "COM1", "\\.\COM2"
     * 
     * @param devname The name of the device. E.g. "ttyS0" or "COM1".
     * @return The port index on success, otherwise a negative value.
     */
    int getPortIdx(const std::string& devName);
}

#endif // RS232_H
