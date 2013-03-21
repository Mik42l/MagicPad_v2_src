#include "objectDetectionFilter.h"

const QString ObjectDetectionFilter::TAG = QString("ObjectDetectionFilter");

/**
 *
 */
ObjectDetectionFilter::ObjectDetectionFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

/**
 *
 */
void ObjectDetectionFilter::process()
{
	static bool prevDetected = false;

    // Compute frame average / frame stddev
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(Frame, mean, stddev);

    //QLOG_TRACE() << TAG << mean[0] << stddev[0];

    if( mean[0] < AverageThreshold ||
        stddev[0] > StdThreshold)
    {
        ObjectDetected = true;
    }
    else
    {
        ObjectDetected = false;
    }

    // Compute obj-background average diff
    //cv::Mat bw;
    //double otsu = cv::threshold(img, bw, -1, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    // TODO: |avg(obj) - avg(bgnd)| * 2 / (avg(obj) + avg(bgnd))

    // Emit enter/exit signal and update internal state
    if(!prevDetected && ObjectDetected) emit objectEnters();
    else if(prevDetected && !ObjectDetected) emit objectLeaves();
    prevDetected = ObjectDetected;
}

/**
 *
 */
void ObjectDetectionFilter::reset()
{
    StdThreshold = OBJECTDETECTION_STDDEV_THRES;
    AverageThreshold = OBJECTDETECTION_AVERAGE_THRES;
    //DiffThreshold = OBJECTDETECTION_DIFF_THRES;

    ObjectDetected = false;
}
