#include "vumeterApplet.h"

const QString VumeterApplet::TAG = QString("VumeterApplet");

/**
 *
 */
VumeterApplet::VumeterApplet(QWidget *parent) : Applet(parent)
{
    mName = VUMETER_NAME;
    mTitle = VUMETER_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // build UI
    mVuMeterWidget = new VuMeterWidget();
    this->setLayout( new QHBoxLayout() );
    this->layout()->addWidget( mVuMeterWidget );

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter();
    mMeanFilter = new MeanFilter();
    mObjectDetectionFilter = new ObjectDetectionFilter();

    // When an object leaves the MagicPad set the vumeter to the (n-10)th sample
    connect(mObjectDetectionFilter, SIGNAL(objectLeaves()), this, SLOT(resurect()));

    // Gestures
    setGestures(UP_DOWN);
}

/**
 *
 */
void VumeterApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    cv::Mat center = cv::Mat(mCalibrationFilter->getCalibratedFrame(), cv::Range(2,8), cv::Range(2,8) );

    mMeanFilter->setFrame( center );
    mMeanFilter->process();

    mObjectDetectionFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mObjectDetectionFilter->process();

    //QLOG_TRACE() << TAG << "detected:" << mObjectDetectionFilter->getObjectDetected();

    // If an object is detected over the sensor then the vumeter value
    // is updated continuously.
    if(mObjectDetectionFilter->getObjectDetected() == true)
    {
        double value = mMeanFilter->getMean() * 100.0 / 255.0;

        // Record sample
        mList.push_front( value );
        while(mList.size() > VUMETER_RESURECT_LENGTH) mList.pop_back();


        // Update vumeter using slightly old value
        mVuMeterWidget->setValue( mList.at( qMin(3, mList.size()/2) ) );
    }

    update();
}

/**
 *
 */
void VumeterApplet::reset()
{
    mCalibrationFilter->reset();
    mMeanFilter->reset();
    mObjectDetectionFilter->reset();

    mList.clear();
}

/**
 *
 */
void VumeterApplet::resurect()
{
    //QLOG_TRACE() << TAG << "resurect()";
    if( !mList.empty() )
    {
        mVuMeterWidget->setValue( mList.back() );
    }
}
