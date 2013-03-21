#include "pongApplet.h"

const QString PongApplet::TAG = QString("PongApplet");

/**
 *
 */
PongApplet::PongApplet(QWidget *parent) : Applet(parent)
{
    mName = PONG_NAME;
    mTitle = PONG_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

	// UI    
    mPongGame = new GraphicsScene(0, 0, 976, 672);
    connect(this, SIGNAL(meanChanged(int)), mPongGame->getBarPlayer(), SLOT(modifyY(int)) );

    QGraphicsView *view = new QGraphicsView;
    view->setScene( mPongGame );

    setLayout( new QHBoxLayout() );
    layout()->addWidget( view );

    mPongGame->setupScene();

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter();
    mMeanFilter = new MeanFilter();
    mObjectDetectionFilter = new ObjectDetectionFilter();
    mMean = 128;
}

/**
 *
 */
void PongApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    cv::Mat center = cv::Mat(mCalibrationFilter->getCalibratedFrame(), cv::Range(2,8), cv::Range(2,8) );
    mMeanFilter->setFrame( center );
    mMeanFilter->process();    

    mObjectDetectionFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mObjectDetectionFilter->process();

    // If an object is detected over the sensor then the vumeter value
    // is updated continuously.
    if(mObjectDetectionFilter->getObjectDetected() == true)
    {
        mMean = mMeanFilter->getMean();
    }
    else
    {
        mMean += ( 128 - mMean ) * 0.1;
    }

    // Map 80->180 to 0->255
    const double MINI = 25.0;
    const double MAXI = 225.0;
    mMean = qBound( MINI, mMean, MAXI );

    emit meanChanged( 255.0 * (mMean-MINI)/(MAXI-MINI) );

    update();
}

/**
 *
 */
void PongApplet::reset()
{
    mCalibrationFilter->reset();
    mMeanFilter->reset();
    mObjectDetectionFilter->reset();
}
