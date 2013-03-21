#include "opdApplet.h"

const QString OpdApplet::TAG = QString("OpdApplet");

/**
 *
 */
OpdApplet::OpdApplet(QWidget *parent) : Applet(parent)
{
    mName = OPD_NAME;
    mTitle = OPD_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Filter pipeline
    mCalibrationFilter = new CalibrationFilter();

    reset();

    // Gestures
    setGestures(SWAP_ALL);
}

/**
 *
 */
void OpdApplet::mousePressEvent( QMouseEvent * event )
{
    if( (event->button() & Qt::LeftButton) || (event->button() & Qt::MiddleButton) )
    {
        cv::imwrite( QString( "raw_%1.png" ).arg( mImageCounter ).toStdString(), mRaw );
    }

    if( (event->button() & Qt::RightButton) || (event->button() & Qt::MiddleButton) )
    {
        cv::imwrite( QString( "cal_%1.png" ).arg( mImageCounter ).toStdString(), mCal );
    }

    mImageCounter++;
}

/**
 * paintEvent - paint the content of the widget
 */
void OpdApplet::paintEvent(QPaintEvent *)
{
    if( mCal.empty() || mRaw.empty() || mRef.empty() ) {
        return;
    }

    QPainter painter( this );
    // draw image
    painter.setPen( Qt::NoPen );

    double w = rect().width() / mCal.cols;
    double h = rect().height() / mCal.rows;

    QRectF px(0, 0, w, h);

    for(int i=0; i<mCal.rows; ++i)
    {
        for(int j=0; j<mCal.cols; ++j)
        {
            unsigned char raw = mRaw.at<unsigned char>(i, j);
            double ref = mRef.at<float>(i, j);
            unsigned char cal = mCal.at<unsigned char>(i, j);
            unsigned char alive = mAlive.at<unsigned char>(i, j);

            if( alive )
            {
                painter.setPen( QColor(Qt::green).darker(150) );
            }
            else
            {
                painter.setPen( Qt::red );
            }

            // Pixel color
            px.moveTo( j*w, i*h );
            painter.setBrush( QColor( cal, cal, cal ) );
            painter.drawRect( px );

            // Pixel values
            painter.drawText( px, " ADC:" + QString::number( raw ) + "\n Max:" + QString::number( ref ) + "\n Cal:" + QString::number( cal ) );
        }
    }
}


/**
 *
 */
void OpdApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mRaw = frame;

    mRef = mCalibrationFilter->getReferenceFrame();

    mCal = mCalibrationFilter->getCalibratedFrame();

    mAlive = mCalibrationFilter->getAlive();

    update();
}

/**
 *
 */
void OpdApplet::reset()
{
    mCalibrationFilter->reset();
    mImageCounter = 1;

    mCal = cv::Mat();
    mRef = cv::Mat();
    mRaw = cv::Mat();

}
