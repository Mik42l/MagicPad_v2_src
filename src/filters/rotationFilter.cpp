#include "rotationFilter.h"

const QString RotationFilter::TAG = QString("RotationFilter");

/**
 *
 */
RotationFilter::RotationFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

/**
 *
 */
void RotationFilter::process()
{
    //QLOG_TRACE() << TAG << "processing() ...";
    computeOpticalFlow();
}

/**
 * Set internal attributes to their original state.
 */
void RotationFilter::reset()
{
    // Setup matrices, and stuff
    mBigFrame = cv::Mat( 30, 30, CV_8U );
    mBigFrame.setTo( cv::Scalar::all(128) );
    Frame = mBigFrame( cv::Range(10, 20), cv::Range(10, 20) );

    mBigPrevious = cv::Mat( 30, 30, CV_8U );
    mBigPrevious.setTo( cv::Scalar::all(128) );
    mPrevious = mBigPrevious( cv::Range(10, 20), cv::Range(10, 20) );

    mBigFlow = cv::Mat( 30, 30, CV_32FC2 );
    mBigFlow.setTo( cv::Scalar::all(0) );
    Flow = mBigFlow( cv::Range(10, 20), cv::Range(10, 20) );

    Damping = 1.0;
    Clamped = false;
    UseKalman = true;

    Moment = 0.0;
    Speed = 0.0;
    Angle = 0.0;
    mTargetAngle = 0.0;

    MomentToSpeed = ROTATION_FILTER_MOMENT_TO_SPEED;
    SpeedToAngle = ROTATION_FILTER_SPEED_TO_ANGLE;

    // Initialize Kalman Filter
    mKalmanFilter = new cv::KalmanFilter( 3, 1, 0 );
    mKalmanFilter->statePre.at<float>( 0 ) = Angle;	 // alpha
    mKalmanFilter->statePre.at<float>( 1 ) = Speed;	 // alpha'
    mKalmanFilter->statePre.at<float>( 2 ) = Moment; // alpha''

    mKalmanFilter->transitionMatrix = *( cv::Mat_<float>(3, 3) <<
        1, 1, 0,    // angle(t+1) ~ angle(t) + speed(t)
        0, 1, 1,    // speed(t+1) ~ speed(t) + moment(t)
        0, 0, 1 );  // moment(t+1) ~ moment(t)

    cv::setIdentity( mKalmanFilter->measurementMatrix );
    cv::setIdentity( mKalmanFilter->processNoiseCov, cv::Scalar::all( 0.001 ) ); // Process Noise Covariance (0.001)
    cv::setIdentity( mKalmanFilter->measurementNoiseCov, cv::Scalar::all( 0.01 ) ); // Measurement Noise Covariance (0.01)
    cv::setIdentity( mKalmanFilter->errorCovPost, cv::Scalar::all( 0.1 ) );      // Error Covariance Post (0.1)
}

/**
 * Compute Optical Flow and Rotation
 */
void RotationFilter::computeOpticalFlow()
{
    // TODO: user objectDetectionFilter instead of dirty hack
    bool objDetect = cv::mean( Frame )[0] < 255*0.90;

    //
    // Optical Flow computation
    //
    cv::calcOpticalFlowFarneback(
                mBigPrevious,       // (current-1)^th frame (input)
                mBigFrame,          // current frame (input)
                mBigFlow,           // optical flow (output)
                0.5,                // pyramid scale (0.5=default)
                2,                  // number of pyramidal levels (1=default)
                3,                  // winsize
                3,                  // iteration
                3,                  // poly neighboor
                1.1,                // poly sigma
                0 & cv::OPTFLOW_FARNEBACK_GAUSSIAN );                 // flags

    // Save the current frame
    // The current frame is 10-by-10. m_previous points to the central area
    // of m_bigPrevious (30-by-30).
    Frame.copyTo( mPrevious );

    // If no object is over the sensor then quit, ie: do not update Angle
    //if( !objDetect ) return;

    //
    // Compute motion
    //
    if( objDetect )
    {
        MotionVector = motion( Flow );
    }
    else
    {
        MotionVector = cv::Point2f(0,0);
    }

    //
    // Compute Angle
    //

    // Moment
    double m = 0.0;
    if( objDetect ) {
        m = moment( Flow );
        //QLOG_DEBUG() << TAG << m;
    }
    Moment = qBound( -500., m, 500. );

    // Speed
    Speed = Moment * MomentToSpeed;

    // Angle
    mTargetAngle += Speed * SpeedToAngle;

    if( Clamped )
    {
        mTargetAngle = qBound( 0.0, mTargetAngle, 360.0 );
    }
    else
    {
       // If angle overflow :
       // while( mTargetAngle >= 360.0 ) mTargetAngle -= 360.0;
       // while( mTargetAngle < 0.0 ) mTargetAngle += 360.0;
    }

    // Use Kalman filter to predict angle position
    if( UseKalman )
    {
        mKalmanFilter->predict();
        cv::Mat measurement(1, 1, CV_32F );
        measurement.at<float>(0) = mTargetAngle;
        cv::Mat estimated = mKalmanFilter->correct( measurement );
        Angle = estimated.at<float>( 0 );
    }
    else
    {
        Angle += ( mTargetAngle - Angle ) * Damping;
    }
}

/**
 * Return the resulting moment of all vectors of 'src'.
 */
double RotationFilter::moment( const cv::Mat& src )
{
    const int BORDER = 0;
    const cv::Point3f center(src.cols/2.0, src.rows/2.0, 0.0);
    cv::Point3f m(0, 0, 0);

    for(int i=BORDER; i<src.rows-BORDER; i++)
    {
        for(int j=BORDER; j<src.cols-BORDER; j++)
        {
            cv::Point2f f = src.at<cv::Point2f>(i,j);
            cv::Point3f F(f.x, f.y, 0);
            cv::Point3f pos(j, i, 0);
            m += F.cross(center-pos);
        }
    }

    return m.z;
}

/**
 * Return the global motion
 */
cv::Point2f RotationFilter::motion( const cv::Mat& src )
{
    cv::Point2f vec(0, 0);

    for(int i=0; i<src.rows; i++)
    {
        for(int j=0; j<src.cols; j++)
        {
            vec += src.at<cv::Point2f>(i,j);
        }
    }

    return vec;
}
