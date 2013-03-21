#ifndef GESTUREAPPLET_H
#define GESTUREAPPLET_H

#include "applet.h"

#include <QPainter>

#include "filters/calibrationFilter.h"
#include "filters/objectDetectionFilter.h"
#include "filters/rotationFilter.h"

#define GESTURE_NAME "gesture"
#define GESTURE_TITLE "Gesture"

#define GESTURE_LIST_SIZE 60

#define GESTURE_MOTION_SPEED 1.0

class GestureApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    GestureApplet(QWidget *parent = 0);

public slots:
    void paintEvent(QPaintEvent *);

    void setFrame( cv::Mat& frame );

    void reset();

private:
    void centerPoints();

private:
    // Filters
    CalibrationFilter *mCalibrationFilter;
    RotationFilter *mRotationFilter;

    // Image
    QList< QPoint > mPoints;
};

#endif // GESTUREAPPLET_H
