#include "gestureApplet.h"

const QString GestureApplet::TAG = QString("GestureApplet");

/**
 *
 */
GestureApplet::GestureApplet(QWidget *parent) : Applet(parent)
{
    mName = GESTURE_NAME;
    mTitle = GESTURE_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Filter pipeline
    mCalibrationFilter = new CalibrationFilter();
    mRotationFilter = new RotationFilter();

    // Gestures
    setGestures(SWAP_ALL, TWIST);
}

/**
 *
 */
void GestureApplet::centerPoints()
{
    if( mPoints.empty() ) return;

    QPoint delta = -mPoints.last() + rect().center();

    QList< QPoint >::iterator it;

    for(it = mPoints.begin(); it != mPoints.end(); ++it)
    {
        *it = *it + delta;
    }
}

/**
 *
 */
void GestureApplet::paintEvent(QPaintEvent *)
{
    QPainter painter( this );

    painter.setRenderHint( QPainter::Antialiasing );

    //
    // Draw lines
    //
    QPen linePen( Qt::blue );
    linePen.setWidthF( 5.0 );

    QPen shadowLinePen( Qt::black );
    shadowLinePen.setWidthF( 10 );

    QRectF nodeRect( QPointF(0, 0), QSizeF(10, 10) );
    QPen nodePen( Qt::black );
    nodePen.setWidthF( 3.0 );
    QBrush nodeBrush( Qt::white );

    if( mPoints.size() > 1 )
    {
        QList< QPoint >::iterator it;
        int n = 0;
        for(it = mPoints.begin(); it != mPoints.end(); ++it)
        {
            QList< QPoint >::iterator next = it + 1;
            if( next == mPoints.end() ) break;

            // Opacity increases for newer samples
            double opactiy = qMin(1.0, 0.15 + (double)n++ / mPoints.size());
            painter.setOpacity( opactiy/2.0 );

            // Draw shadow first
            painter.setPen( shadowLinePen );
            painter.drawLine(*it, *next);

            // Draw line
            painter.setOpacity( opactiy );
            painter.setPen( linePen );
            painter.drawLine(*it, *next);

            // Draw connection between lines
            painter.setPen( nodePen );
            painter.setBrush( nodeBrush );
            nodeRect.moveCenter( *it );
            painter.drawEllipse( nodeRect );
        }
    }
}

/**
 *
 */
void GestureApplet::setFrame( cv::Mat& frame )
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mRotationFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mRotationFilter->process();

    cv::Point2f motion = mRotationFilter->getMotionVector();

    // Add new point
    if( mPoints.empty() )
    {
        mPoints.append( rect().center() );
    }
    else
    {
        mPoints.append( mPoints.last() + GESTURE_MOTION_SPEED * QPoint(motion.x, motion.y) );
    }

    // Resize mPoints if too big
    while(mPoints.size() > GESTURE_LIST_SIZE)
    {
        mPoints.removeFirst();
    }

    // Center the points if out of frame
    if( !rect().contains( mPoints.last() ) )
    {
        centerPoints();
    }

    update();
}

/**
 *
 */
void GestureApplet::reset()
{
    mCalibrationFilter->reset();
    mRotationFilter->reset();

    mPoints.clear();
}
