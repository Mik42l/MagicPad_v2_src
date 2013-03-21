#ifndef OPDAPPLET_H
#define OPDAPPLET_H

#include "applet.h"

#include <QPainter>
#include <QMouseEvent>

#include "filters/calibrationFilter.h"

#define OPD_NAME "opd"
#define OPD_TITLE "Opd"

/**
 *
 */
class OpdApplet : public Applet
{

    Q_OBJECT

public:    

    static const QString TAG;

    OpdApplet(QWidget *parent = 0);

public slots:

    void mousePressEvent(QMouseEvent * event);

    void paintEvent(QPaintEvent *);

    void setFrame( cv::Mat& frame );

    void reset();

private:
    CalibrationFilter *mCalibrationFilter;

    cv::Mat mRaw, mRef, mCal, mAlive;

    int mImageCounter;
};

#endif // OPDAPPLET_H
