#ifndef OTSUFILTER_H
#define OTSUFILTER_H

#include "filter.h"

/**
 * OtsuFilter 
 */
class OtsuFilter : public Filter
{
public:
    static const QString TAG;

    OtsuFilter(QObject *parent = 0);

    // Input Frame
    FILTER_SET_MACRO(Frame, const cv::Mat&)

    // mean value of input image
    FILTER_GET_MACRO(Threshold, double)
	
	//
    FILTER_GET_MACRO(Mask, const cv::Mat&)

public slots:
    void process();
    void reset();

protected slots:

private:
    double Threshold;
    cv::Mat Frame;
    cv::Mat Mask;
};

#endif // OTSUFILTER_H
