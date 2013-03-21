#include "switchApplet.h"

const QString SwitchApplet::TAG = QString("SwitchApplet");

/**
 * Constructor: setup filters & UI
 */
SwitchApplet::SwitchApplet(QWidget *parent) :
    Applet(parent),
    mSoundBip( SWITCH_SOUND_BIP, this )
{
    mName = SWITCH_NAME;
    mTitle = SWITCH_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    //
    // Build filter pipeline
    //
    mCalibrationFilter = new CalibrationFilter( this );

    mSwitchFilter = new SwitchFilter( this );
    connect(mSwitchFilter, SIGNAL( toggle() ), this, SLOT( toggle() ) );

    //
    // GUI & Internal state
    //
    resize( parent->size() );

    mSwitchState = false;
    mDim = 0.0;

    QPropertyAnimation *turnOnAnimation = new QPropertyAnimation( this, "dim" );
    turnOnAnimation->setDuration(200);
    turnOnAnimation->setStartValue(0.0);
    turnOnAnimation->setEndValue(1.0);
    connect(this, SIGNAL(turnLightOn()), turnOnAnimation, SLOT(start()));

    QPropertyAnimation *turnOffAnimation = new QPropertyAnimation( this, "dim" );
    turnOffAnimation->setDuration(200);
    turnOffAnimation->setStartValue(1.0);
    turnOffAnimation->setEndValue(0.0);
    connect(this, SIGNAL(turnLightOff()), turnOffAnimation, SLOT(start()));

    // Gestures
    setGestures(SWAP_LEFT_RIGHT);
}

/**
 * Paint the switch
 */
void SwitchApplet::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint( QPainter::Antialiasing );

    // Draw background (if any)
    //painter.fillRect(rect(), Qt::blue);

    // Set viewport to a square area centered
    int sz = qMin( rect().width(), rect().height() );
    QRect vp(0, 0, sz, sz);
    vp.moveCenter( rect().center() );


    // Draw background
    QRadialGradient bgGradient(vp.center(), sz/2);
    bgGradient.setColorAt(0.7, QColor(0, 0, 0, 255) );
    bgGradient.setColorAt(1.0, QColor(0, 0, 0, 0) );
    //painter.fillRect( vp, Qt::red );
    painter.fillRect( vp, bgGradient );

    // Draw the switch
    painter.translate( vp.topLeft() );

    // Color varies between Qt::lightGray(0xc0, 0xc0, 0xc0) and Qt::red(0xff, 0, 0)
    QColor color(
        0xc0 + (0xff-0xc0)*mDim,
        0xc0 * (1.0-mDim),
        0xc0 * (1.0-mDim) );

    //QPen pen( mSwitchState ? Qt::red : Qt::lightGray );
    QPen pen( color );
    pen.setWidthF( 35.0 + 5.0*mDim );
    pen.setCapStyle( Qt::RoundCap );
    painter.setPen( pen );

    QLine line1(sz/2, sz/2, sz/2, sz/6 );
    painter.drawLine( line1 );

    QRect round(sz/4, sz/4, sz/2, sz/2);
    painter.drawArc( round, 120*16, 300*16  );
}

/**
 * Process new input frame
 */
void SwitchApplet::setFrame(cv::Mat &frame)
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();
    double m = cv::mean( mCalibrationFilter->getCalibratedFrame() )[0];

    mSwitchFilter->setInput( m );
    mSwitchFilter->process();
    // mSwitchFilter is connected to (this, toogle())

    update();
}

/**
 * Change the switch state
 */
void SwitchApplet::toggle()
{
    if( mSwitchState )
    {
        turnOff();
    }
    else
    {
        turnOn();
    }
}

/**
 * Turn ON the switch
 */
void SwitchApplet::turnOn()
{
    mSoundBip.play();
    mSwitchState = true;
    emit turnLightOn();
}

/**
 * Turn OFF the switch
 */
void SwitchApplet::turnOff()
{
    mSoundBip.play();
    mSwitchState = false;
    emit turnLightOff();
}
