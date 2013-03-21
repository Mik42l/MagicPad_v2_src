#ifndef TWISTAPPLET_H
#define TWISTAPPLET_H

#include "applet.h"

#include <QDial>
#include <QGridLayout>
#include <QPushButton>

#include "filters/calibrationFilter.h"
#include "filters/delayFilter.h"
#include "filters/objectDetectionFilter.h"
#include "filters/rotationFilter.h"

#include "widgets/oventimerWidget.h"
#include "widgets/opticalflowWidget.h"

#define TWIST_NAME "twist"
#define TWIST_TITLE "Twist"

class TwistApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    TwistApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void setTargetAngle() {
        if( mDelayFilter->isOutputReady() ) {
            mTarget = mDelayFilter->getOutput();
            //QLOG_DEBUG() << TAG << "setting mTarget to" << mTarget;
        }
        else
        {
            mTarget = mAngle;
            //QLOG_DEBUG() << TAG << "delay not ready, mTarget set to" << mTarget;
        }
    }

signals:

    void angleChanged(int);

    void flowChanged( const cv::Mat& );

    void frameChanged( const cv::Mat& );

private:
    CalibrationFilter *mCalibrationFilter;
    DelayFilter< double > *mDelayFilter;
    ObjectDetectionFilter *mObjectDetectionFilter;
    RotationFilter *mRotationFilter;

    double mTarget;
    double mAngle;

    // GUI
    QDial *mKnob;
    OvenTimerWidget *mOvenTimer;
    OpticalFlowWidget *mOpticalFlow;
};

#endif // TWISTAPPLET_H
