#include "common/SerialIO.hpp"

bool getBaudrateCode(int i_baudrate, int * i_baudrate_code){
	// baudrate codes are defined in termios.h
	// currently upto B1000000
    const int baud_table[]= {
        0, 50, 75, 110, 134, 150, 200, 300, 600,
		1200, 1800, 2400, 4800,
		9600, 19200, 38400, 57600, 115200, 230400,
		460800, 500000, 576000, 921600, 1000000
    };

    const int baud_codes[] = {
		B0, B50, B75, B110, B134, B150, B200, B300, B600,
		B1200, B1800, B2400, B4800,
		B9600, B19200, B38400, B57600, B115200, B230400,
		B460800, B500000, B576000, B921600, B1000000
	};

    const int i_bauds_len = sizeof(baud_table) / sizeof(int);

	bool ret = false;
	*i_baudrate_code = B38400;
	int i;
	for( i=0; i<i_bauds_len; i++ ) {
		if( baud_table[i] == i_baudrate ) {
			*i_baudrate_code = baud_codes[i];
			ret = true;
			break;
		}
    }

	return ret;
}

SerialIO::SerialIO() 
    : _device_name{""},
      _device{-1},
      _baud_rate{115200},
      _multiplier{1.0},
      _byte_size{8},
      _stop_bits{StopBits::SB_ONE},
      _parity{ParityFlags::PA_NONE},
      _handshake{HandshakeFlags::HS_NONE},
      _read_buf_size{1024},
      _write_buf_size{_read_buf_size},
      _timeout{0},
      _short_byte_period{false}
{
    _byte_period.tv_sec = 0;
    _byte_period.tv_usec = 0;
}

SerialIO::~SerialIO(){
    closeIO();
}

void SerialIO::setDeviceName(const std::string& name){ _device_name = name;}

void SerialIO::setBaudRate(int baud_rate){ _baud_rate = baud_rate;}

void SerialIO::setMultiplier(double multiplier = 1.0){ _multiplier = multiplier;}

void SerialIO::setHandshake(HandshakeFlags handshake){ _handshake = handshake;}

    
void SerialIO::purge()
{
    ::tcflush(_device, TCIOFLUSH);
}

void SerialIO::purgeRx() 
{
    tcflush(_device, TCIFLUSH);
}

void SerialIO::purgeTx() 
{
	tcflush(_device, TCOFLUSH);
}

void SerialIO::flushTx() 
{
	tcdrain(_device);
}


void SerialIO::setFormat(int byte_size, ParityFlags parity, StopBits stop_bits)
{   _byte_size = byte_size;
    _parity = parity;
    _stop_bits = stop_bits;
}

void SerialIO::setBufferSize(int read_buf_size, int write_buf_size)
{   
    _read_buf_size = read_buf_size;
    _write_buf_size = write_buf_size;
}

void SerialIO::setTimeout(double timeout)
{
	_timeout = timeout;
	if (_device != -1)
	{
		_tio.c_cc[VTIME] = cc_t(ceil(_timeout * 10.0));
		tcsetattr(_device, TCSANOW, &_tio);
	}

}

void SerialIO::setBytePeriod(double period)
{
    _short_byte_period = false;
    _byte_period.tv_sec = time_t(period);
	_byte_period.tv_usec = suseconds_t((period - _byte_period.tv_sec) * 1000);
}


int SerialIO::openIO()
{
    int res;
    //open device
    _device = open(_device_name.c_str(), O_RDWR | O_NOCTTY);

    if(_device<0){
        std::cout<<"Trying to open" << _device_name<< " failed: "
        <<strerror(errno)<< " (Error code )"<<errno<<") \n";
    }
    return -1;
//set parameters

res = tcgetattr(_device, &_tio);
    if(res == -1){
        std::cout<< "tcgetattr of "<<_device_name<< " failed "
        <<strerror(errno) <<" (Error code "<< errno<<") \n";

        close(_device);
        _device = -1;
        return -1;
    }

    //default values
	_tio.c_iflag = 0;
	_tio.c_oflag = 0;
	_tio.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	_tio.c_lflag = 0;
	cfsetispeed(&_tio, B9600);
	cfsetospeed(&_tio, B9600);

	_tio.c_cc[VINTR] = 3;	// Interrupt
	_tio.c_cc[VQUIT] = 28;	// Quit
	_tio.c_cc[VERASE] = 127;	// Erase
	_tio.c_cc[VKILL] = 21;	// Kill-line
	_tio.c_cc[VEOF] = 4;	// End-of-file
	_tio.c_cc[VTIME] = 0;	// Time to wait for data (tenths of seconds)
	_tio.c_cc[VMIN] = 1;	// Minimum number of characters to read
	_tio.c_cc[VSWTC] = 0;
	_tio.c_cc[VSTART] = 17;
	_tio.c_cc[VSTOP] = 19;
	_tio.c_cc[VSUSP] = 26;
	_tio.c_cc[VEOL] = 0;	// End-of-line
	_tio.c_cc[VREPRINT] = 18;
	_tio.c_cc[VDISCARD] = 15;
	_tio.c_cc[VWERASE] = 23;
	_tio.c_cc[VLNEXT] = 22;
	_tio.c_cc[VEOL2] = 0;	// Second end-of-line


    //set baud rate

    int i_new_baudrate = static_cast<int>(_baud_rate * _multiplier + 0.5);
    std::cout<<"setting baudrate to"<<i_new_baudrate<<"\n";
    int i_baudrate_code = 0;
    bool b_baud_rate_valid = getBaudrateCode(i_new_baudrate, &i_baudrate_code);

    cfsetispeed(&_tio, i_baudrate_code);
    cfsetospeed(&_tio, i_baudrate_code);

    if (!b_baud_rate_valid){
     
     std::cout<< "Baudrate code not available - sttting baudrate directly"<<"\n";
     struct serial_struct ss;
     ioctl(_device,TIOCGSERIAL, &ss);
     ss.flags |= ASYNC_SPD_CUST;
     ss.custom_divisor = ss.baud_base / i_new_baudrate;
     ioctl(_device, TIOCSSERIAL, &ss);
    }

    //set data format 
    _tio.c_cflag &= ~CSIZE;
    switch (_byte_size)
	{
		case 5:
			_tio.c_cflag |= CS5;
			break;
		case 6:
			_tio.c_cflag |= CS6;
			break;
		case 7:
			_tio.c_cflag |= CS7;
			break;
		case 8:
		default:
			_tio.c_cflag |= CS8;
	}
    _tio.c_cflag &= ~ (PARENB | PARODD);

    switch (_parity)
	{
		case ParityFlags::PA_ODD:
			_tio.c_cflag |= PARODD;
			[[fallthrough]];
			//break;  // break must not be active here as we need the combination of PARODD and PARENB on odd parity.

		case ParityFlags::PA_EVEN:
			_tio.c_cflag |= PARENB;
			break;

		case ParityFlags::PA_NONE:
		default: {}
	}

    switch (_stop_bits)
	{
		case StopBits::SB_TWO:
			_tio.c_cflag |= CSTOPB;
			break;

		case StopBits::SB_ONE:
		default:
			_tio.c_cflag &= ~CSTOPB;
	}

    // hardware handshake
	switch (_handshake)
	{
		case HandshakeFlags::HS_NONE:
			_tio.c_cflag &= ~CRTSCTS;
			_tio.c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		case HandshakeFlags::HS_HARDWARE:
			_tio.c_cflag |= CRTSCTS;
			_tio.c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		case HandshakeFlags::HS_XONXOFF:
			_tio.c_cflag &= ~CRTSCTS;
			_tio.c_iflag |= (IXON | IXOFF | IXANY);
			break;
	}

    _tio.c_oflag &= ~OPOST;
	_tio.c_lflag &= ~ICANON;

	// write parameters
	res = tcsetattr(_device, TCSANOW, &_tio);

	if (res == -1)
	{
		std::cout << "tcsetattr " << _device_name << " failed: "
			<< strerror(errno) << " (Error code " << errno << ")" << std::endl;

		close(_device);
		_device = -1;

		return -1;
	}

	// set timeout
	setTimeout(_timeout);

	return 0;

}

void SerialIO::closeIO()
{
	if (_device != -1)
	{
		close(_device);
		_device = -1;
	}
}



int SerialIO::readBlocking(char *Buffer, int Length)
{
	ssize_t bytes_read;
	bytes_read = read(_device, Buffer, Length);
#ifdef PRINT_BYTES
	printf("%2d Bytes read:", bytes_read);
	for(int i=0; i<bytes_read; i++)
		printf(" %.2x", (unsigned char)Buffer[i]);
	printf("\n");
#endif
	return bytes_read;
}

int SerialIO::readNonBlocking(char *Buffer, int Length)
{
	int iAvaibleBytes = getSizeRXQueue();
	int iBytesToRead = (Length < iAvaibleBytes) ? Length : iAvaibleBytes;
	ssize_t bytes_read;


	bytes_read = read(_device, Buffer, iBytesToRead);
	return bytes_read;
}

int SerialIO::writeIO(const char *Buffer, int Length)
{
	ssize_t bytes_written;

	if (_byte_period.tv_usec || _byte_period.tv_sec)
	{
		int i;
		for (i = 0; i < Length; i++)
		{
			bytes_written = write(_device, Buffer + i, 1);
			if (bytes_written != 1)
				break;
			select(0, 0, 0, 0, &_byte_period);
		}
		bytes_written = i;
	}
	else
		bytes_written = write(_device, Buffer, Length);
#ifdef PRINT_BYTES
	printf("%2d Bytes sent:", bytes_written);
	for(int i=0; i<bytes_written; i++)
		printf(" %.2x", (unsigned char)Buffer[i]);
	printf("\n");
#endif

	return bytes_written;
}

int SerialIO::getSizeRXQueue()
{
	int cbInQue;
	int Res = ioctl(_device, FIONREAD, &cbInQue);
	if (Res == -1) {
		return 0;
	}
	return cbInQue;
}



