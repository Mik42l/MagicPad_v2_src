#ifndef MEANFILTER_H
#define MEANFILTER_H

#include "filter.h"

/**
 * MeanFilter 
 */
class MeanFilter : public Filter
{
public:
    static const QString TAG;

    MeanFilter(QObject *parent = 0);

    // Input Frame
    FILTER_SET_MACRO(Frame, const cv::Mat&)

    // mean value of input image
    FILTER_GET_MACRO(Mean, double)

public slots:

    void process();

    void reset();

protected slots:

private:
    double Mean;
    cv::Mat Frame;
};

#endif // MEANFILTER_H
