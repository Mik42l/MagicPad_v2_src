#include "calibrationFilter.h"

const QString CalibrationFilter::TAG = QString( "CalibrationFilter" );

/**
 *
 */
CalibrationFilter::CalibrationFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

void CalibrationFilter::reset()
{
    // Default filter values
    Decay = 0.5; //Default = .85
    Periodicity = 60 ; //Default = 20
    mPeriodCounter = 0;
}

/**
 *
 */
void CalibrationFilter::computeReferenceFrame()
{
    // Update maximum        
    cv::max(ReferenceFrame * Decay, mFrame32F, ReferenceFrame);

    // Prevent ReferenceFrame from having 0-value pixels
    // that would cause errors during calculation
    cv::max(ReferenceFrame, 1.0, ReferenceFrame);

    // Check dead/alive pixels
    Alive.setTo( cv::Scalar::all(255) );
    Alive = ReferenceFrame > CALIBRATIONFILTER_ALIVE_THRESHOLD;

    // D19 diodes are always dead
    Alive.at<unsigned char>(3, 3) = 0;
    Alive.at<unsigned char>(3, 6) = 0;
    Alive.at<unsigned char>(6, 3) = 0;
    Alive.at<unsigned char>(6, 6) = 0;

    // 0:dead, 1:alive
    Alive /= 255;
}

/**
 * Interpolate D19 pixel values
 */
void CalibrationFilter::interpolate()
{
    // Gaussian interpolation mask
    float coeff[9] = {
        .707,  1.0, .707,
         1.0,  0.0,  1.0,
        .707,  1.0, .707 };
    const cv::Mat mask(3, 3, CV_32F, coeff);

    for(int i=1; i<mFrame32F_big.rows-1; ++i)
    {
        for(int j=1; j<mFrame32F_big.cols-1; ++j)
        {
            // Check if pixel is dead
            if(mAlive_big.at<unsigned char>(i, j) == 0)
            {
                // Dead pixel: build interpolation matrix
                cv::Mat neighbor( 3, 3, CV_32F );
                mAlive_big( cv::Range(i-1, i+2), cv::Range(j-1, j+2) ).convertTo( neighbor, CV_32F );

                //QLOG_DEBUG() << TAG << "neighbor sum" << cv::sum( neighbor )[0];

                // Compute interpolated value
                // 'M' is the same as 'mask' but with 0-coefficient over dead pixels
                const cv::Mat M = mask.mul( neighbor, 1.0 );

                // 'roi' Region of interest is the 3x3 region centered on the dead pixel
                const cv::Mat roi = mFrame32F_big( cv::Range(i-1, i+2), cv::Range(j-1, j+2) );
                double sum = cv::sum( M )[0];
                if(sum >= 1.0)
                {
                    mFrame32F_big.at<float>(i, j) = M.dot( roi ) / sum;
                }
                else
                {
                    mFrame32F_big.at<float>(i, j) = 0;
                }
            }
        }
    }
}

/**
 *
 */
void CalibrationFilter::process()
{
    // build reference frame if needed
    if(( mFrame32F.empty() ) || ( mFrame32F.size != Frame.size ) )
    {
        mFrame32F_big = cv::Mat( Frame.rows+2, Frame.cols+2, CV_32F, cv::Scalar::all(0) );
        mFrame32F = mFrame32F_big( cv::Range( 1, Frame.rows+1 ), cv::Range( 1, Frame.cols+1 ) );
    }
    Frame.convertTo(mFrame32F, CV_32F);

    if(( ReferenceFrame.empty() ) || ( ReferenceFrame.size != Frame.size ) )
    {
        //QLOG_TRACE() << TAG << "building reference frame";
        ReferenceFrame = cv::Mat( Frame.rows, Frame.cols, CV_32F );

        mAlive_big = cv::Mat( Frame.rows+2, Frame.cols+2, CV_8U, cv::Scalar::all(0) );
        Alive = mAlive_big( cv::Range( 1, Frame.rows+1 ), cv::Range( 1, Frame.cols+1 ) );

        Frame.convertTo( ReferenceFrame, CV_32F );
    }

    // Run the 'max' algorithm once every 'Periodicity' calls
    if(mPeriodCounter == 0)
    {
        this->computeReferenceFrame();
    }
    mPeriodCounter = (mPeriodCounter+1) % Periodicity;

    // Calibrate frame mFrame32F = (mFrame32F * 255) / ReferenceFrame
    cv::divide(mFrame32F, ReferenceFrame, mFrame32F, 255.0);

    // Interpolate dead/defective pixels
    interpolate();

    // Final convertion to <unsigned char> type
    mFrame32F.convertTo(CalibratedFrame, CV_8U);
}
