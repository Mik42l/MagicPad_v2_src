#ifndef FRAMEPRODUCER_H
#define FRAMEPRODUCER_H

#include <QThread>

#include "com/magicPad.h"

/**
 *
 */
class FrameProducer : public QThread
{

    Q_OBJECT

public:

    FrameProducer();

    void run();

    void setDevice(MagicPadDevice& device);

signals:

    void newFrame(const cv::Mat & frame, long unsigned int timestamp);
    //void newFrame(cv::Mat frame);
    void newFrame( cv::Mat & frame );

public slots:

    void getNewFrame(MagicPadFrame *frame);

private:

    MagicPadDevice mDevice;

    bool mFirstFrame;

    cv::Mat mReferenceFrame;
};

#endif // FRAMEPRODUCER_H
