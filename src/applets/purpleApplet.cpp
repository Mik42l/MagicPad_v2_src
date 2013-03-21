#include "purpleApplet.h"

const QString PurpleApplet::TAG = QString("PurpleApplet");

/**
 *
 */
PurpleApplet::PurpleApplet(QWidget *parent) : Applet(parent)
{
    mName = PURPLE_NAME;
    mTitle = PURPLE_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter();
    mObjectDetectionFilter = new ObjectDetectionFilter();
    mJoystickFilter = new JoystickFilter();
    mPidFilter = new PidFilter<cv::Point2f>();
    reset();

    // Gestures
    setGestures(SWAP_ALL);
}

/**
 *
 */
void PurpleApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mObjectDetectionFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mObjectDetectionFilter->process();

    mJoystickFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mJoystickFilter->process();

    cv::Point2f p(0, 0);
    if(mObjectDetectionFilter->getObjectDetected() == true)
    {
        // Update position + low pass filter
        p = mJoystickFilter->getPosition();
        mPos += (p-mPos)*0.75;
    }
    else
    {
        // when no object is detected, slowly set the joystick to the central position
        mPos *= 0.8;
    }

    update();
}

/**
 *
 */
void PurpleApplet::reset()
{
    mCalibrationFilter->reset();
    mObjectDetectionFilter->reset();
    mJoystickFilter->reset();

    mPidFilter->reset();
    mPidFilter->initialize(cv::Point2f(0, 0));
    mPidFilter->setProportional(0.4);
    mPidFilter->setIntegral(0.0);
    mPidFilter->setDerivative(0.0);

    mPos.x = 0;
    mPos.y = 0;
}

/**
 *
 */
void PurpleApplet::paintEvent(QPaintEvent *)
{
    QPainter painter( this );
    // squares dimensions and color
    const double mvt_coeff[4] = {0.1, 0.25, 0.6, 1.2};
    const double mvt = 1.25;

    const QSizeF S = geometry().size();
    const double size_coeff[4] = {0.8, 1.1, 1.6, 2.2};
    const double size = 0.15 * std::min( S.width(), S.height() );

    QColor color[4];
    color[0] = QColor(0.50*255, 0.20*255, 0.4*255);
    color[1] = QColor(0.65*255, 0.25*255, 0.6*255);
    color[2] = QColor(0.85*255, 0.30*255, 0.8*255);
    color[3] = QColor(1.00*255, 0.35*255, 1.0*255);

    // drawing the 4 squares, furthest to nearest
    QPen pen;
    for(int i=0; i<4; i++)
    {        
        pen.setColor( QColor(color[i].blue(),color[i].blue(),color[i].blue()) );
        pen.setWidthF( size_coeff[i]/size_coeff[0] );
        painter.setPen( pen );
        painter.setBrush( color[i] );

        QPointF center( S.width()/2, S.height()/2 );
        center += mvt * mvt_coeff[i] * 0.3 * QPointF( mPos.x * S.width(), mPos.y * S.height() );
        QPointF s = QPointF( 0.5, 0.5 ) * size * size_coeff[i];
        QRectF r( center - s, center + s );
        painter.drawRoundedRect( r, 10.0, 10.0, Qt::RelativeSize );
    }
}
