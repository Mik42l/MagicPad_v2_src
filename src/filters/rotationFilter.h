#ifndef ROTATIONFILTER_H
#define ROTATIONFILTER_H

#include "filter.h"

#define ROTATION_FILTER_MOMENT_TO_SPEED 1.0
#define ROTATION_FILTER_SPEED_TO_ANGLE 0.25

/**
 * RotationFilter 
 */
class RotationFilter : public Filter
{
    Q_OBJECT	
public:
    static const QString TAG;

    RotationFilter( QObject *parent = 0 );

    void computeOpticalFlow();

    double moment( const cv::Mat& src );

    cv::Point2f motion( const cv::Mat& src );

    // Clamped means that Angle stays between 0.0 and 360.0
    FILTER_SET_MACRO(Clamped, bool)

    // Damping coefficient to smooth out results
    // Must be 0< Damping <= 1.0
    FILTER_SET_MACRO(Damping, double)

    // Use kalaman filter
    FILTER_SET_MACRO(UseKalman, bool)

    // Input image
    //FILTER_SET_MACRO(Frame, const cv::Mat&)
    void setFrame(const cv::Mat& _arg)
    {
        _arg.copyTo( Frame );
    }

    // Moment To Speed coefficient
    FILTER_SET_MACRO(MomentToSpeed, double)

    // Speed To Angle coefficient
    FILTER_SET_MACRO(SpeedToAngle, double)

    // Computed Moment
    FILTER_GET_MACRO(Moment, double)

    // Computed Speed
    FILTER_GET_MACRO(Speed, double)

    // Computed Angle
    FILTER_GET_MACRO(Angle, double)

    // Optical flow
    FILTER_GET_MACRO(Flow, const cv::Mat&)

    // Motion Vector
    FILTER_GET_MACRO(MotionVector, cv::Point2f)

public slots:
    void process();
    void reset();

private:
    // Interface
    double Damping;
    bool Clamped;
    cv::Mat Frame;

    double Moment;
    double Speed;
    double Angle;
    double mTargetAngle;

    double MomentToSpeed;
    double SpeedToAngle;

    cv::Point2f MotionVector;

    // Optical Flow
    cv::Mat mBigFrame;    // last frame from sensor
    cv::Mat mBigPrevious; // previous m_frame
    cv::Mat mBigFlow;     // optical flow

    cv::Mat mPrevious;     // previous m_frame
    cv::Mat Flow;         // optical flow

    // Kalman Filter
    cv::KalmanFilter *mKalmanFilter;
    bool UseKalman;
};

#endif // ROTATIONFILTER_H
