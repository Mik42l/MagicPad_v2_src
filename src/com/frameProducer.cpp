#include "frameProducer.h"

/**
 * Constructor
 */
FrameProducer::FrameProducer()
    : mDevice(0, "")
{

}

/**
 * This method is called once using thread->start();
 */
void FrameProducer::run()
{
    QTimer *timer = new QTimer( );
    timer->setInterval(40);

    MagicPad *magicPad = new MagicPad();
    if(!magicPad->connectDevice(mDevice))
    {
        QLOG_ERROR() << "Unable to connect to MagicPad on COM port" << mDevice.comport;
    }

    connect(timer, SIGNAL(timeout()), magicPad, SLOT(getDeviceFrame()));
    connect(magicPad, SIGNAL(newFrame(MagicPadFrame*)), this, SLOT(getNewFrame(MagicPadFrame*)));

    timer->start();

    // Run the event loop
    exec();
}

/**
 * Set the magicPad device
 */
void FrameProducer::setDevice(MagicPadDevice& device)
{
    mDevice = device;
}

/**
 *
 */
void FrameProducer::getNewFrame(MagicPadFrame *frame)
{
    // send new frame to listener
    //emit newFrame( frame->img, frame->timestamp );
    emit newFrame( frame->img );

    // compute producer framerate
    static int frameCnt = 0;
    static unsigned long int prevTime = 0;
    if(frameCnt == 24)
    {
        //double fps = (25 * 1000) / (frame->timestamp - prevTime);
        //emit framerateChanged( fps );
        frameCnt = 0;
        prevTime = frame->timestamp;
    }
    else
    {
        frameCnt++;
    }
}
