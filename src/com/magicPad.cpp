#include "magicPad.h"

/**
 * Constructor
 */
MagicPad::MagicPad(QObject *parent)
    : QObject(parent), TAG("MagicPad")
{
    mDeviceHandle = INVALID_HANDLE_VALUE;
}

/**
 * Connect to a specified COM port
 */
bool MagicPad::connectDevice(const MagicPadDevice& device)
{
    if(mDeviceHandle != INVALID_HANDLE_VALUE) {
        disconnectDevice();
    }

    if(!openComPort(device.comport, mDeviceHandle))
    {
        QLOG_WARN() << TAG << "Could not open COM port " << device.comport << "for device " << device.name;
        disconnectDevice();
        return false;
    }

#ifndef MAGICPAD_OLD_FIRMWARE
    if(!checkDevice(mDeviceHandle))
    {
        QLOG_WARN() << TAG << "Check device failed: " << device.comport << device.name;
        disconnectDevice();
        return false;
    }
#endif

    return true;
}

/**
 * Disconnect from current device
 */
void MagicPad::disconnectDevice()
{
    CloseHandle(mDeviceHandle);

    mDeviceHandle = INVALID_HANDLE_VALUE;
}

/**
 * Find all MagicPad devices connected
 */
QList<MagicPadDevice> MagicPad::listDevices()
{
    QList<MagicPadDevice> devices;

    devices.append( this->listDevicesFTDI() );
    devices.append( this->listDevicesBluetooth());

    return devices;
}

/**
 *
 */
QList<MagicPadDevice> MagicPad::listDevicesFTDI()
{
    QList<MagicPadDevice> devicesPortName;

    FT_STATUS ftStatus;
    DWORD numDevs;
    FT_HANDLE fthandle;
    LONG comport;

    // === List FTDI devices ===
    // Get the number of devices currently connected
    ftStatus = FT_ListDevices(&numDevs, NULL, FT_LIST_NUMBER_ONLY);
    if(ftStatus != FT_OK)
    {
        // FT_ListDevices failed
        QLOG_ERROR() << TAG << "FT_ListDevices failed :" << ftStatus;
        return devicesPortName;
    }

    // FT_ListDevices OK, number of devices connected is in numDevs
    for(int i=0; i<numDevs; ++i)
    {
        // Get the COM port number
        ftStatus = FT_Open(i, &fthandle);
        if(ftStatus != FT_OK){
            FT_Close(fthandle);
            QLOG_TRACE() << TAG << "opening " << i << " failed! with error " << ftStatus;
            continue;
        }
        ftStatus = FT_GetComPortNumber(fthandle, &comport);
        if(ftStatus != FT_OK){
            FT_Close(fthandle);
            QLOG_WARN() << TAG << "get com port failed " << ftStatus;
            continue;
        }
        if(comport == -1) {
            FT_Close(fthandle);
            QLOG_WARN() << TAG << "no com port installed";
            continue;
        }       

        FT_Close(fthandle);

        QLOG_INFO() << TAG << "Opened COM port" << comport << "succesfully";

        // Check that device is a MagicPad
        HANDLE h;
        if(!openComPort(comport, h)) {
            CloseHandle(h);
            QLOG_WARN() << TAG << "Could not open COM port" << comport;
            continue;
        }

#ifdef MAGICPAD_OLD_FIRMWARE
        devicesPortName.append(MagicPadDevice(comport, "MagicPad #?"));
#else
        if(checkDevice(h))
        {
            QString name = getDeviceName(h);
            QLOG_INFO() << TAG << "Device id=" << i << "on COM port" << comport << "is" << name;
            devicesPortName.append(MagicPadDevice(comport, name));
        }
        else
        {
            QLOG_INFO() << TAG << "Device id=" << i << "on COM port" << comport << "is NOT a MagicPad";
        }
#endif
        CloseHandle(h);
    }

    return devicesPortName;
}

/**
 *
 */
QList<MagicPadDevice> MagicPad::listDevicesBluetooth()
{
    QList<MagicPadDevice> devicesPortName;

    // list COM port

    // check devices

    return devicesPortName;
}

/**
 *
 */
bool MagicPad::checkDevice(HANDLE& h)
{
    // send 'CHECK' command
    if(!writeDevice(h, "?", 1)) return false;

    // check response
    char buf[1];
    if(!readDevice(h, buf, 1)) return false;

    if(buf[0] != 42) // 42 is the magic number that identifies MagicPads
    {
        QLOG_WARN() << TAG << "checkDevice failed: wrong answer:" << (int)buf[0];
        return false;
    }

    return true;
}

/**
 *
 */
QString MagicPad::getDeviceName(HANDLE &h)
{
    // send 'NAME' command
    if(!writeDevice(h, "N", 1)) return "#error";

    // read name
    char buf[16];
    if(!readDevice(h, buf, 16)) return "#error";

    return QString(buf);
}

/**
 *
 */
bool MagicPad::writeDevice(HANDLE& h, const char* data, long len)
{
    BOOL fSuccess;
    DWORD dwwritten = 0;

    fSuccess = WriteFile(h, data, len, &dwwritten, NULL);
    if(!fSuccess)
    {        
        QLOG_WARN() << TAG << "Write Failed " << reportError(GetLastError());
        QLOG_ERROR() << TAG << "writeDevice() failed " << h;
        return false;
    }

    if( len != dwwritten )
    {
        QLOG_WARN() << TAG << dwwritten << " bytes written," << len << "requested";
        return false;
    }

    return true;
}

/**
 *
 */
bool MagicPad::readDevice(HANDLE& h, char* data, long len)
{
    BOOL fSuccess;
    DWORD dwRead;
    //memset(buf,0,256);

    fSuccess = ReadFile(h, data, len, &dwRead, NULL);

    if(!fSuccess)
    {
        QLOG_WARN() << TAG << "data read = " << data << ", error:" << reportError(GetLastError());
        return false;
    }

    if( len != dwRead )
    {
        QLOG_WARN() << TAG << dwRead << " bytes read," << len << "requested";
        return false;
    }

    return true;
}

/**
 * Open the given com port and set the handle to the file descriptor
 */
bool MagicPad::openComPort(LONG comport, HANDLE& h)
{
    char COMx[16];
    DCB dcb;
    BOOL fSuccess;
    COMMTIMEOUTS TimeOut;

    sprintf(COMx, "\\\\.\\COM%d", comport);
    h = CreateFile(
            COMx,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL );

    if(h == INVALID_HANDLE_VALUE)
    {
        QLOG_WARN() << TAG << "Help - failed to open " << COMx;
        return false;
    }

    // Configure the UART interface parameters
    fSuccess = GetCommState(h, &dcb);
    if(!fSuccess)
    {
        QLOG_WARN() << TAG << "GetCommState Failed" << reportError(GetLastError());
        return false;
    }

    // Set parameters
    dcb.BaudRate = MAGICPAD_BAUDRATE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    fSuccess = SetCommState(h, &dcb);
    if(!fSuccess)
    {
        QLOG_WARN() << TAG << "SetCommState Failed" << reportError(GetLastError());
        return false;
    }

    // Set timeout
    TimeOut.ReadIntervalTimeout = MAXDWORD;
    TimeOut.ReadTotalTimeoutMultiplier = 1;
    TimeOut.ReadTotalTimeoutConstant = 50; // 100 miliseconds
    TimeOut.WriteTotalTimeoutMultiplier = 1;
    TimeOut.WriteTotalTimeoutConstant = 50;
    if(!SetCommTimeouts(h, &TimeOut))
    {
        QLOG_WARN() << TAG << "SetCommTimeouts Failed: " << reportError(GetLastError());
        return false;
    }

    return true;
}

/**
 *
 */
void MagicPad::getDeviceFrame()
{
#ifdef MAGICPAD_OLD_FIRMWARE
        // OLD FRAME FORMAT

        // send 'FRAME' command
        if(!writeDevice(mDeviceHandle, "A", 1)) return;

        // read name
        unsigned char buf[102];
        if(!readDevice(mDeviceHandle, (char *)buf, 102)) return;

        MagicPadFrame *frame = new MagicPadFrame( buf+1, 1 );

        emit newFrame(frame);
#else
        // NEW FRAME FORMAT

        // send 'FRAME' command
        if(!writeDevice(mDeviceHandle, "B", 1)) return;

        // read name
        unsigned char buf[106];

        if(!readDevice(mDeviceHandle, (char *)buf, 106)) return;

        unsigned long int timestamp = (buf[1] << 24) + (buf[2] << 16) + (buf[3] << 8) + (buf[4]);

        MagicPadFrame *frame = new MagicPadFrame(buf+5, timestamp);

        emit newFrame(frame);
#endif
}

/**
 *
 */
QString MagicPad::reportError(DWORD dwError)
{
    const DWORD dwFormatControl = FORMAT_MESSAGE_ALLOCATE_BUFFER |
                                  FORMAT_MESSAGE_IGNORE_INSERTS |
                                  FORMAT_MESSAGE_FROM_SYSTEM;

    LPVOID pTextBuffer = NULL;
    DWORD dwCount = FormatMessage(dwFormatControl,
                                  NULL,
                                  dwError,
                                  0,
                                  (LPTSTR) &pTextBuffer,
                                  0,
                                  NULL);
    if(0 != dwCount)
    {
        QString s((char *)pTextBuffer);
        LocalFree(pTextBuffer);
        return s;
    }
    else
    {
        return QString("Unknown error");
    }
}
