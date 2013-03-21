#ifndef BACKLIGHT_H
#define BACKLIGHT_H

#include "applet.h"

#include <QPainter>

#include "filters/calibrationFilter.h"
#include "filters/meanFilter.h"

#define BACKLIGHT_NAME "backlight"
#define BACKLIGHT_TITLE "Backlight"

// Background image
#define BACKLIGHT_BACKGROUND_IMAGE ":image/backlight.jpg"

class BacklightApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    BacklightApplet(QWidget *parent = 0);

public slots:

    void paintEvent(QPaintEvent *);

    void setFrame( cv::Mat& frame );

    void reset();

private:
    CalibrationFilter *mCalibrationFilter;
    cv::Mat mFrame;
    cv::Mat mSmooth;

    QImage *mOverlay;
    QImage mBackground;
};

#endif // BACKLIGHT_H
