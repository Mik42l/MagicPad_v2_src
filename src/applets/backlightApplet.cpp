#include "backlightApplet.h"

const QString BacklightApplet::TAG = QString("BacklightApplet");

/**
 *
 */
BacklightApplet::BacklightApplet(QWidget *parent) : Applet(parent)
{
    mName = BACKLIGHT_NAME;
    mTitle = BACKLIGHT_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Filter pipeline
    mCalibrationFilter = new CalibrationFilter();
    mOverlay = new QImage(10, 10, QImage::Format_ARGB32);
    mFrame = cv::Mat(10, 10, CV_8U);
    mSmooth = cv::Mat(10, 10, CV_8U);
    reset();

    // UI
    if( !mBackground.load( BACKLIGHT_BACKGROUND_IMAGE ) )
    {
        QLOG_ERROR() << TAG << "Unable to load" << BACKLIGHT_BACKGROUND_IMAGE;
    }

    // Gestures
    setGestures(SWAP_ALL);
}

/**
 *
 */
void BacklightApplet::paintEvent(QPaintEvent *)
{
    QPainter painter( this );

    // Draw background
    painter.drawImage( rect(), mBackground );

    // Draw overlay
    // 10x10 input image is scaled up to a 64x64 image, using smoothing transformation to
    // prevent aliasing effect. It is then scaled to be drawn onto the surface
    painter.drawImage(
        rect(),
        (*mOverlay).scaled( QSize(128, 128), Qt::IgnoreAspectRatio, Qt::SmoothTransformation),
        QRect(0,0,128,128));
}

/**
 *
 */
void BacklightApplet::setFrame( cv::Mat& frame )
{
    // Filters
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    // Temporal low-pass filter
    cv::GaussianBlur( mCalibrationFilter->getCalibratedFrame(), mSmooth, cv::Size(3, 3), 0.0 );

    for(int i=0; i<mSmooth.rows; i++)
    {
        for(int j=0; j<mSmooth.cols; j++)
        {
            int frameValue = mFrame.at<unsigned char>(i, j);
            int smoothValue = mSmooth.at<unsigned char>(i, j);
            if( qAbs(frameValue-smoothValue) <= 1 )
            {
                mFrame.at<unsigned char>(i, j) = smoothValue;
            }
            else
            {
                frameValue = mFrame.at<unsigned char>(i, j) = frameValue + (smoothValue-frameValue) * 0.2;
            }

            unsigned char bw = frameValue > 170 ? 255 : 0;
            mOverlay->setPixel( j, i, qRgba(bw, bw, bw, std::abs(frameValue-170)*1.2) );
        }
    }

    update();
}

/**
 *
 */
void BacklightApplet::reset()
{
    mCalibrationFilter->reset();
    mFrame.setTo( cv::Scalar::all(128) );
}
