#ifndef VUMETERAPPLET_H
#define VUMETERAPPLET_H

#include "applet.h"

#include <QHBoxLayout>
#include <QList>

#include "filters/calibrationFilter.h"
#include "filters/meanFilter.h"
#include "filters/objectDetectionFilter.h"

#include "widgets/vumeterWidget.h"

#define VUMETER_NAME "vumeter"
#define VUMETER_TITLE "Vu-meter"

// Delay in # of frames
#define VUMETER_RESURECT_LENGTH 15

/**
 *
 */
class VumeterApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    VumeterApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void resize() {
        mVuMeterWidget->resize( size() );
    }

    void reset();

    // resurect last valid input from user
    void resurect();

private:
    CalibrationFilter *mCalibrationFilter;
    MeanFilter *mMeanFilter;
    ObjectDetectionFilter *mObjectDetectionFilter;

    VuMeterWidget *mVuMeterWidget;
    QList<double> mList;
};

#endif // VUMETERAPPLET_H
