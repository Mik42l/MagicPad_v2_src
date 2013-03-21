#include "otsuFilter.h"

const QString OtsuFilter::TAG = QString("OtsuFilter");

/**
 *
 */
OtsuFilter::OtsuFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

/**
 *
 */
void OtsuFilter::process()
{
    if(Mask.size != Frame.size )
    {
        Mask = cv::Mat(Frame.size(), Frame.type() );
    }
    Threshold = cv::threshold( Frame, Mask, -1, 255, CV_THRESH_BINARY | CV_THRESH_OTSU );
}

/**
 *
 */
void OtsuFilter::reset()
{
    Mask = cv::Mat();
}
