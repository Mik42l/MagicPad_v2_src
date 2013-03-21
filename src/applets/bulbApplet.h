#ifndef BULBAPPLET_H
#define BULBAPPLET_H

#include "applet.h"

#include <QPainter>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QGridLayout>

#include "filters/calibrationFilter.h"

#define BULB_NAME "bulb"
#define BULB_TITLE "Smart Bulb"

// Background image
#define BULB_BACKGROUND_IMAGE ":image/backlight.jpg"
#define BULB_DIM_TEMPO_IN 20    // time before entering 'dimming' mode
#define BULB_DIM_TEMPO_OUT 15   // time before leaving 'dimming' mode
#define BULB_DIM_INC 4          // increment speed when dimming the light
#define BULB_SWITCH_TIME 10      // speed of fade-in fade-out effet when swhitching on/off the light

/**
 *
 */
class BulbApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    BulbApplet(QWidget *parent = 0);

public slots:

    void paintEvent(QPaintEvent *);

    void setFrame( cv::Mat& frame );

    void reset();

private:
    void setLight(int value) {
        mLightSlider->setValue( value );
    }

private:

    CalibrationFilter *mCalibrationFilter;

    cv::Mat mFrame;
    cv::Mat mBlackWhite;

    QImage mBackground;

    enum { IDLE, SWITCHING_OFF, SWITCHING_ON, DIMMING } mState;
    cv::Mat mBw;
    int mLightValue;   // light pwm value
    bool mLightState; // false: light off; true: light on

    // GUI
    QPushButton *mOnOffButton;
    QPushButton *mPlusButton;
    QPushButton *mMinusButton;
    QSlider *mLightSlider;
    QLabel *lbl; // debug label
};

#endif // BULBAPPLET_H
