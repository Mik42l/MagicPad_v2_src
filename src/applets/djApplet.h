#ifndef DJAPPLET_H
#define DJAPPLET_H

#include "applet.h"

#include <QPainter>

#include "filters/calibrationFilter.h"
#include "filters/rotationFilter.h"

#define DJ_NAME "dj"
#define DJ_TITLE "DJ"

class DjApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    DjApplet(QWidget *parent = 0);

public slots:

    void paintEvent(QPaintEvent *);

    void setFrame( cv::Mat& frame );

private:

    CalibrationFilter *mCalibrationFilter;
    RotationFilter *mRotationFilter;

    double mSpeed;
    double mAngle;
    QPixmap mDiskImage;

};

#endif // DJAPPLET_H
