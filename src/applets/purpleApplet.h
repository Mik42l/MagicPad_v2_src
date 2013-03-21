#ifndef PURPLEAPPLET_H
#define PURPLEAPPLET_H

#include "applet.h"

#include <QPainter>

#include "filters/calibrationFilter.h"
#include "filters/joystickFilter.h"
#include "filters/objectDetectionFilter.h"
#include "filters/pidFilter.h"

#define PURPLE_NAME "purple"
#define PURPLE_TITLE "Purple"

class PurpleApplet : public Applet
{
    Q_OBJECT
public:
    static const QString TAG;

    PurpleApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void paintEvent(QPaintEvent *);

    void resize() {
        //mVuMeterWidget->resize( this->size().toSize() );
    }

    void reset();

private:
    CalibrationFilter *mCalibrationFilter;
    JoystickFilter *mJoystickFilter;
    ObjectDetectionFilter *mObjectDetectionFilter;
    PidFilter<cv::Point2f> *mPidFilter;

    cv::Point2f mPos;
};

#endif // PURPLEAPPLET_H
