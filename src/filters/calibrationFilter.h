#ifndef CALIBRATIONFILTER_H
#define CALIBRATIONFILTER_H

#include "filter.h"

#include <opencv2/core/core.hpp>

#define CALIBRATIONFILTER_ALIVE_THRESHOLD 15

/**
 * CalibrationFilter
 */
class CalibrationFilter : public Filter
{
public:
    static const QString TAG;

    CalibrationFilter(QObject *parent = 0);

    // Damping coefficent
    FILTER_SET_GET_MACRO(Decay, double)

    // Periodicity: number of frames between reference update
    FILTER_SET_GET_MACRO(Periodicity, int)

    // Input Frame
    FILTER_SET_MACRO(Frame, const cv::Mat&)

    // Output calibrated frame
    FILTER_GET_MACRO(CalibratedFrame, const cv::Mat&)

    // Reference frame
    FILTER_GET_MACRO(ReferenceFrame, const cv::Mat&)

    // Alive Map
    FILTER_GET_MACRO(Alive, const cv::Mat&)

public slots:

    void process();

    void reset();

protected slots:

    void computeReferenceFrame();

private:

    void interpolate();

private:
    double Decay;

    int Periodicity;
    int mPeriodCounter;

    cv::Mat Frame;

    cv::Mat mFrame32F_big;
    cv::Mat mFrame32F;

    cv::Mat ReferenceFrame;

    cv::Mat Alive;
    cv::Mat mAlive_big;


    cv::Mat CalibratedFrame;
};

#endif // CALIBRATIONFILTER_H
