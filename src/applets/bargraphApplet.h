#ifndef BARGRAPHAPPLET_H
#define BARGRAPHAPPLET_H

#include "applet.h"

#include <QPainter>

#include "filters/calibrationFilter.h"

#define BARGRAPH_NAME "bargraph"
#define BARGRAPH_TITLE "Bargraph"

/**
 * Bargraph applet displays a 3D volume that can be compressed
 */
class BargraphApplet : public Applet
{

    Q_OBJECT

public:

    static const QString TAG;

    BargraphApplet(QWidget *parent = 0);

public slots:

    void paintEvent(QPaintEvent *);

    void setFrame( cv::Mat& frame );

private:
    void drawBar(QPainter *painter, int x, int y, int l, int h);

private:

    CalibrationFilter *mCalibrationFilter;

    cv::Mat mFrame;

    /*** ***/
    double mAlpha;          // Alpha angle for rotation
    double mBeta;           // Beta angle for rotation
    double mScaleFactor;    // ! level on the sensor = x pixel
    int mBarWidth;          // width of the bar
    int mBarSpacing;        // spacing between two bars
    int mBarSpacing_y;      // spacing between two bars
    int mBarMargin;         // spacing between the window edge and the first/lst bar
    int mBarWindowH;        // heigth of the window
    int mBarWindowW;        // width of the window
    int mOrigin;
};

#endif // BARGRAPHAPPLET_H
