#ifndef PONGAPPLET_H
#define PONGAPPLET_H

#include "applet.h"

#include <QGraphicsView>
#include <QHBoxLayout>

#include "filters/calibrationFilter.h"
#include "filters/meanFilter.h"
#include "filters/objectDetectionFilter.h"

#include "pong/graphicsscene.h"

#define PONG_NAME "pong"
#define PONG_TITLE "Pong"

/**
 *
 */
class PongApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    PongApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void reset();

signals:

    void meanChanged(int mean);

private:
    CalibrationFilter *mCalibrationFilter;
    MeanFilter *mMeanFilter;
    ObjectDetectionFilter *mObjectDetectionFilter;

    GraphicsScene *mPongGame;

    double mMean;
};

#endif // PONGAPPLET_H
