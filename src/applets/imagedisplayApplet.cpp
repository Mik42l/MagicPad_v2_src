#include "imagedisplayApplet.h"

const QString ImagedisplayApplet::TAG = QString("ImageDisplay");

/**
 *
 */
ImagedisplayApplet::ImagedisplayApplet(QWidget *parent) : Applet(parent)
{
    mName = IMAGEDISPLAY_NAME;
    mTitle = IMAGEDISPLAY_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Filters pipeline
    mCalibrationFilter = new CalibrationFilter();
    mFrame = cv::Mat();

    // Gestures
    setGestures(SWAP_ALL);
}

/**
 * paintEvent - paint the content of the widget
 */
void ImagedisplayApplet::paintEvent(QPaintEvent *)
{
    if( mFrame.empty() ) {
        return;
    }

    QPainter painter( this );

    // Placeholder drawing
    painter.setPen( Qt::red );
    painter.setBrush( Qt::black );
    painter.fillRect( rect(), Qt::black );

    // draw image
    painter.setPen( Qt::transparent );
    double w = rect().width() / mFrame.cols;
    double h = rect().height() / mFrame.rows;
    for(int i=0; i<mFrame.rows; ++i)
    {
        for( int j=0; j<mFrame.cols; ++j )
        {
            QRect r( j*w, i*h, w, h );
            unsigned char c = mFrame.at<unsigned char>( i, j );
            painter.setBrush( QColor( c, c, c ) );
            painter.drawRect( r );
            //painter.drawText( r, "  " + QString::number( c ) );
        }
    }
}

/**
 *
 */
void ImagedisplayApplet::setFrame( cv::Mat& frame )
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();
    mFrame = mCalibrationFilter->getCalibratedFrame();

    update();
}
