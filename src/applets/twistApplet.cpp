#include "twistApplet.h"

const QString TwistApplet::TAG = QString("TwistApplet");

/**
 *
 */
TwistApplet::TwistApplet(QWidget *parent) : Applet(parent)
{
    mName = TWIST_NAME;
    mTitle = TWIST_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    //
    // Build filter pipeline
    //
    mCalibrationFilter = new CalibrationFilter( this );

    mDelayFilter = new DelayFilter<double>( this );
    mDelayFilter->setDelay( 15 );

    mObjectDetectionFilter = new ObjectDetectionFilter( this );
    connect( mObjectDetectionFilter, SIGNAL(objectLeaves()), this, SLOT(setTargetAngle()) );

    mRotationFilter = new RotationFilter( this );
    mRotationFilter->setClamped( true );
    mRotationFilter->setDamping( 1.0 );
    mRotationFilter->setUseKalman( true );

    mTarget = 0.0;

    //
    // Build UI
    //
    QGridLayout *layout = new QGridLayout();
    layout->setSpacing(40.0);
    this->setLayout(layout);

    // Buttons
    QPushButton *btnKnob = new QPushButton("\nKnob\n");
    QPushButton *btnTimer = new QPushButton("\nOven timer\n");
    QPushButton *btnFlow = new QPushButton("\nOptical Flow\n");

    btnKnob->setPalette( QPalette(Qt::transparent) );
    btnTimer->setPalette( QPalette(Qt::transparent) );
    btnFlow->setPalette( QPalette(Qt::transparent) );

    btnKnob->setStyleSheet("QPushButton { color: black; }");
    btnTimer->setStyleSheet("QPushButton { color: black; }");
    btnFlow->setStyleSheet("QPushButton { color: black; }");

    layout->addWidget( btnKnob, 1, 1 );
    layout->addWidget( btnTimer, 1, 3 );
    layout->addWidget( btnFlow, 1, 5 );

    // QDial
    mKnob = new QDial();
    mKnob->setRange( 0, 360 );
    mKnob->setWrapping( false );
    mKnob->setNotchesVisible( true );
    QPalette palette = mKnob->palette();
    palette.setColor( mKnob->backgroundRole(), Qt::transparent );
    mKnob->setPalette(palette);
    connect( this, SIGNAL(angleChanged(int)), mKnob, SLOT(setValue(int)) );
    layout->addWidget( mKnob, 2, 1, 1, 5 );

    // OvenTimer
    mOvenTimer = new OvenTimerWidget( this );
    mOvenTimer->setRange( 0, 360 );
    mOvenTimer->hide();
    connect( this, SIGNAL(angleChanged(int)), mOvenTimer, SLOT(setValue(int)) );
    layout->addWidget( mOvenTimer, 2, 1, 1, 5 );

    // OpticalFlow
    mOpticalFlow = new OpticalFlowWidget( this );
    mOpticalFlow->hide();
    connect( this, SIGNAL(flowChanged(const cv::Mat&)), mOpticalFlow, SLOT(setFlow(const cv::Mat&)) );
    connect( this, SIGNAL(frameChanged(const cv::Mat&)), mOpticalFlow, SLOT(setFrame(const cv::Mat&)) );
    layout->addWidget( mOpticalFlow, 2, 1, 1, 5 );

    // Knob signals
    connect( btnKnob, SIGNAL(clicked()), mKnob, SLOT(show()) );
    connect( btnKnob, SIGNAL(clicked()), mOvenTimer, SLOT(hide()) );
    connect( btnKnob, SIGNAL(clicked()), mOpticalFlow, SLOT(hide()) );

    // Oven Timer signals
    connect( btnTimer, SIGNAL(clicked()), mOvenTimer, SLOT(show()) );
    connect( btnTimer, SIGNAL(clicked()), mKnob, SLOT(hide()) );
    connect( btnTimer, SIGNAL(clicked()), mOpticalFlow, SLOT(hide()) );

    // Optical Flow signals
    connect( btnFlow, SIGNAL(clicked()), mOpticalFlow, SLOT(show()) );
    connect( btnFlow, SIGNAL(clicked()), mOvenTimer, SLOT(hide()) );
    connect( btnFlow, SIGNAL(clicked()), mKnob, SLOT(hide()) );

    // Gestures
    setGestures(TWIST);
}

/**
 *
 */
void TwistApplet::setFrame( cv::Mat& frame )
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mObjectDetectionFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mObjectDetectionFilter->process();

    mRotationFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mRotationFilter->process();

    emit flowChanged( mRotationFilter->getFlow() );
    emit frameChanged( mCalibrationFilter->getCalibratedFrame() );

    if( mObjectDetectionFilter->getObjectDetected() )
    {
        mAngle = mRotationFilter->getAngle();
        mDelayFilter->setInput( mAngle );
        emit angleChanged( mAngle );
    }
    else
    {
        mAngle += (mTarget - mAngle) * 0.4;
        emit angleChanged( mAngle );
    }

    //QLOG_DEBUG() << TAG << mRotationFilter->getAngle();

    update();
}
