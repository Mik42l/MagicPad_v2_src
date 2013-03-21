#include "meanFilter.h"

const QString MeanFilter::TAG = QString("MeanFilter");

/**
 *
 */
MeanFilter::MeanFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

/**
 *
 */
void MeanFilter::process()
{
    Mean = cv::mean(Frame)[0];
    //emit MeanChanged(Mean);
}

/**
 *
 */
void MeanFilter::reset()
{
    // Default filter values
    Mean = 0.0;
    Frame = cv::Mat();
}
