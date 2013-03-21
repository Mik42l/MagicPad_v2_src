#ifndef MAGICPAD_H
#define MAGICPAD_H

#include <QtCore>

#include "QsLog.h"

#include <opencv2/opencv.hpp>

#include "magicPadFrame.h"

#include <windows.h>
#include "ftd2xx.h"

#define MAGICPAD_BAUDRATE   115200

#define MAGICPAD_OLD_FIRMWARE

/**
 *
 */
class MagicPadDevice
{
public:
    MagicPadDevice(int _port, const QString& _name)
    {
        name = _name;
        comport = _port;
    }

    QString name;
    int comport;
};


/**
 *
 */
class MagicPad : public QObject
{
    Q_OBJECT

public:
    MagicPad(QObject *parent = 0);

    // List all MagicPad devices that are connected
    QList<MagicPadDevice> listDevices();

    // Connect to a magicpad device
    bool connectDevice(const MagicPadDevice& device);
    void disconnectDevice();

protected:
    // List FTDI devices
    QList<MagicPadDevice> listDevicesFTDI();
    QList<MagicPadDevice> listDevicesBluetooth();

    // Check device integrity
    bool checkDevice(HANDLE& h);

    // Get device name
    QString getDeviceName(HANDLE& h);

    bool writeDevice(HANDLE& h, const char* data, long len);
    bool readDevice(HANDLE& h, char* data, long len);

    bool openComPort(LONG comport, HANDLE& h);
    bool readComPort();
    bool writeComPort();

    QString reportError(DWORD dwError);

signals:
    void newFrame(MagicPadFrame *frame);

public slots:
    void getDeviceFrame();

private:
    HANDLE mDeviceHandle;
    QString TAG;
};

#endif // MAGICPAD_H
