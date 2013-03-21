#ifndef FINGERTIPFILTER_H
#define FINGERTIPFILTER_H

#include "filter.h"

/**
 * FingertipFilter
 */
class FingertipFilter : public Filter
{
    Q_OBJECT

public:

    static const QString TAG;

    FingertipFilter( QObject *parent = 0 );

    FILTER_SET_MACRO(Frame, const cv::Mat&)

public slots:

    void process();

    void reset();

private:

private:
    cv::Mat Frame;
};

#endif // FINGERTIPFILTER_H
