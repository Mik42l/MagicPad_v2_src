#include "bulbApplet.h"

const QString BulbApplet::TAG = QString("BulbApplet");

/**
 *
 */
BulbApplet::BulbApplet(QWidget *parent) : Applet(parent)
{
    mName = BULB_NAME;
    mTitle = BULB_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    //
    // Filter pipeline
    //
    mCalibrationFilter = new CalibrationFilter();
    reset();

    //
    // UI
    //
    if( !mBackground.load( BULB_BACKGROUND_IMAGE ) )
    {
        QLOG_ERROR() << TAG << "Unable to load" << BULB_BACKGROUND_IMAGE;
    }

    // GUI
    QGridLayout *layout = new QGridLayout( );

    mOnOffButton = new QPushButton("ON / OFF");
    layout->addWidget(mOnOffButton, 2, 1, 2, 2);
    mOnOffButton->setCheckable(true);
    mOnOffButton->setChecked(false);

    mPlusButton = new QPushButton("+");
    layout->addWidget( mPlusButton, 2, 3, 1, 1 );
    mPlusButton->setCheckable(true);
    mPlusButton->setChecked(false);

    mMinusButton = new QPushButton("-");
    layout->addWidget( mMinusButton, 3, 3, 1, 1 );
    mMinusButton->setCheckable( true );
    mMinusButton->setChecked( false );

    mLightSlider = new QSlider;
    layout->addWidget( mLightSlider, 1, 1, 1, 3 );
    mLightSlider->setRange( 0, 100 );
    mLightSlider->setOrientation( Qt::Horizontal );

    lbl = new QLabel();
    layout->addWidget(lbl, 4, 1, 1, 1);

    // Logic
    mState = IDLE;
    mLightState = 0;
    mLightValue = 50;
    setLight(mLightValue);

    // Gestures
    setGestures(SWAP_LEFT_RIGHT, TOP_RIGHT, BOTTOM_RIGHT);

}

/**
 *
 */
void BulbApplet::paintEvent(QPaintEvent *)
{
    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing, true );

    // sould we add light or darken the image ?
    // the middle value is set to 170 instead of 128 because
    // it is harder to cover the sensor than letting it blank
    int lum = (255*mLightValue)/100;
    if(!mLightState) lum = 0;
    unsigned char bw = lum > 220 ? 255 : 0;

    // draw background image
    painter.drawImage( rect(), mBackground );

    // draw semi-transparent overlay on top of background image
    painter.setBrush( QColor(bw, bw, bw, std::abs(lum-220)*1.0) );
    painter.drawRect( rect() );

    // draw interaface
    //   overlay: rounder rectangle
    QRect s = rect();
    s.setTop( s.bottom() - s.height()/3 );
    s.setLeft( s.right() - 2*s.width()/3 );
    s.setWidth( s.width()/2 );
    painter.setViewport( s );
    painter.setWindow( 0, 0, 100, 100 );

    //painter.save();
    QPainterPath rounded_rect;
    rounded_rect.addRoundRect( 0, 0, 100, 100, 5, 5 );
    painter.setClipPath( rounded_rect );
    painter.fillPath( rounded_rect,QBrush( Qt::black ) );
    //painter.restore();

    QColor bright(255, 0, 0);
    QColor dim(100, 0, 0);
    QColor dark(25, 0, 0);
    QColor c;

    //   power
    if(mLightState) {
        painter.setPen(bright);
        painter.setBrush(bright);
    }else{
        painter.setPen(dark);
        painter.setBrush(dark);
    }
    painter.drawEllipse(10, 40, 20, 20);

    //   up/down
    QPolygon poly;
    poly.append(QPoint());
    QFont f("Courier", 24, QFont::Bold);
    painter.setFont(f);
    if( mPlusButton->isChecked() ) {
        c = bright;
    } else if(mState == DIMMING) {
        c = dim;
    }else {
        c = dark;
    }
    painter.setPen(c);
    painter.setBrush(c);
    painter.drawText(66, 10, 30, 30, Qt::AlignCenter, "+");

    if( mMinusButton->isChecked() ) {
        c = bright;
    } else if( mState == DIMMING ) {
        c = dim;
    } else {
        c = dark;
    }
    painter.setPen(c);
    painter.setBrush(c);
    painter.drawText(66, 66, 30, 30, Qt::AlignCenter, "-");
}


/**
 *
 */
void BulbApplet::setFrame( cv::Mat& frame )
{
    // Filters
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();
    mFrame = mCalibrationFilter->getCalibratedFrame();

    bool detected = false;
    bool up = false;
    bool down = false;

    // on/off detection
    double thres = cv::threshold(mFrame, mBlackWhite, -1, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    if(thres<200 && thres>2) {
        detected = true;
        // plus/minus detection
        up = cv::norm(mBlackWhite(cv::Range(0, 3), cv::Range(0, 9)), cv::NORM_L1) > 20;
        down = cv::norm(mBlackWhite(cv::Range(6, 9), cv::Range(0, 9)), cv::NORM_L1) > 20 &&
               cv::norm(mBlackWhite(cv::Range(0, 5), cv::Range(0, 9)), cv::NORM_L1) < 20;
    }

    // state machine
    mPlusButton->setChecked(false);
    mMinusButton->setChecked(false);

    static int idleCounter = 0;
    static int switchingCounter = 0;
    static int dimmingCounter = 0;

    switch(mState)
    {
    case IDLE:
        lbl->setText("IDLE");
        if(detected) {
            idleCounter++;
            if(idleCounter >= BULB_DIM_TEMPO_IN) {
                idleCounter = 0;
                mLightState = true;
                mOnOffButton->setChecked(true);
                mState = DIMMING;
            }
        } else {
            if(idleCounter > 0) {
                idleCounter = 0;
                if(mLightState) {
                    mState = SWITCHING_OFF;
                    mLightState = false;
                    mOnOffButton->setChecked(false);
                    mOnOffButton->setStyleSheet("QPushButton {background : gray}");
                } else {
                    mState = SWITCHING_ON;
                    mLightState = true;
                    mOnOffButton->setChecked(true);
                    mOnOffButton->setStyleSheet("QPushButton {background : red}");
                }
            }
        }
        break;

    case SWITCHING_ON:
        lbl->setText("SWITCHING_ON");
        switchingCounter++;
        if(switchingCounter > BULB_SWITCH_TIME) {
            switchingCounter = 0;
            mState = IDLE;
        }
        break;

    case SWITCHING_OFF:
        lbl->setText("SWITCHING_OFF");
        switchingCounter++;
        if(switchingCounter>BULB_SWITCH_TIME) {
            switchingCounter = 0;
            mState = IDLE;
        }
        break;

    case DIMMING:
        lbl->setText("DIMMING");
        if(detected) {
            dimmingCounter = 0;
            if(up) {
                mPlusButton->setChecked(up);
                mLightValue = std::min(100, mLightValue+4);
            }else if(down) {
                mMinusButton->setChecked(down);
                mLightValue = std::max(0, mLightValue-4);
            }
            setLight(mLightValue);
        }
        else
        {
            dimmingCounter++;
            if(dimmingCounter > BULB_DIM_TEMPO_OUT) {
                dimmingCounter = 0;
                mState = IDLE;
            }
        }
        break;
    }

    update();
}

/**
 *
 */
void BulbApplet::reset()
{
    mCalibrationFilter->reset();
}
