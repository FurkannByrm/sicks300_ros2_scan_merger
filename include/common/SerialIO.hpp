#ifndef _SERIALIO_HPP
#define _SERIALIO_HPP

#include <termios.h>
#include <sys/select.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string>
#include <string.h>

/// Constants for defining the handshake.
    enum class HandshakeFlags{
        HS_NONE,
        HS_HARDWARE,
        HS_XONXOFF
    };

/// Constants for defining the parity bits.

    enum class ParityFlags{
        PA_NONE,
        PA_EVEN,
        PA_ODD,
// UNIX serial drivers only support even, odd, and no parity bit generation.
    };

    enum class StopBits{
        SB_ONE,
        SB_ONE_5,
        SB_TWO
    };
/**
 * Wrapper class for serial communication.
 */
class SerialIO {

    public:

    SerialIO();

	virtual ~SerialIO()noexcept;

    void setDeviceName(const std::string& name);

    void setBaudRate(int baud_rate);   

    void setMultiplier(double multiplier); 

    void setFormat(int byte_size, ParityFlags parity, StopBits stop_bits);

   	void setHandshake(HandshakeFlags handshake);

    void setBufferSize(int read_buf_size, int write_buf_size);

    void setTimeout(double timeout);

    void setBytePeriod(double period);

    int openIO();

    void closeIO();

    int readBlocking(char *buffer, int length);

    int readNonBlocking(char *buffer, int length);

    int writeIO(const char *buffer, int length);

    int getSizeRXQueue();

    void purge();
    	
    void purgeRx();
	
    void purgeTx();

	void flushTx();


    protected:

    ::termios _tio;
    std::string _device_name;
    int _device;
    int _baud_rate;
    double _multiplier;
    int _byte_size;
    StopBits _stop_bits;
    ParityFlags _parity;
    HandshakeFlags _handshake;
    int _read_buf_size, _write_buf_size;
    double _timeout;
    ::timeval _byte_period;
    bool _short_byte_period; 

};




#endif //_SERIALIO_HPP