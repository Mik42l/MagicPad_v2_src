#include "djApplet.h"

const QString DjApplet::TAG = QString("DjApplet");

/**
 *
 */
DjApplet::DjApplet(QWidget *parent) : Applet(parent)
{
    mName = DJ_NAME;
    mTitle = DJ_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Filter pipeline
    mCalibrationFilter = new CalibrationFilter( this );
    mRotationFilter = new RotationFilter( this );
    mRotationFilter->setClamped( false );

    mAngle = 0.0;
    mSpeed = 0.0;
    mDiskImage = QPixmap(":/image/dj/disk.png");

    // Gestures
    setGestures(TWIST);
}

/**
 *
 */
void DjApplet::paintEvent(QPaintEvent *)
{
    QPainter painter( this );

    int sz = qMin( qMin( height(), width() ), mDiskImage.width() );

    painter.translate( width()/2, height()/2 );
    painter.rotate( mAngle );
    painter.translate( -sz/2, -sz/2 );

    painter.drawPixmap( 0, 0, sz, sz, mDiskImage );
}

/**
 *
 */
void DjApplet::setFrame(cv::Mat &frame)
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mRotationFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mRotationFilter->process();

    double moment = qBound(-200.0, mRotationFilter->getMoment(), 200.0);

    mSpeed += moment * 0.20;
    mSpeed = qBound(-50.0, mSpeed, +50.0);

    mAngle += mSpeed * 0.5;

    //QLOG_DEBUG() << TAG << moment << mSpeed << mAngle;

    mSpeed *= 0.88;

    update();
}
