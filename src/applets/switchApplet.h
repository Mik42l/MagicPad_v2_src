#ifndef SWITCHAPPLET_H
#define SWITCHAPPLET_H

#include "applet.h"

#include <QPainter>
#include <QSound>
#include <QPropertyAnimation>

#include "filters/calibrationFilter.h"
#include "filters/switchFilter.h"

#define SWITCH_NAME "switch"
#define SWITCH_TITLE "Switch"

#define SWITCH_SOUND_BIP    "data/sound/oven-bip.wav"

class SwitchApplet : public Applet
{
    Q_OBJECT

    Q_PROPERTY(double dim READ getDim WRITE setDim)

public:

    static const QString TAG;

    SwitchApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void toggle();

    void paintEvent(QPaintEvent *);

    void setDim(double percent)
    {
        mDim = percent;
    }
    double getDim()
    {
        return mDim;
    }

signals:
    void turnLightOn();
    void turnLightOff();

private:

    void turnOn();

    void turnOff();

private:

    CalibrationFilter *mCalibrationFilter;

    SwitchFilter *mSwitchFilter;

    bool mSwitchState;

    QSound mSoundBip;

    double mDim;

};

#endif // SWITCHAPPLET_H
