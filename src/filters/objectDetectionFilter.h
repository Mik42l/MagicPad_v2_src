#ifndef OBJECTDETECTIONFILTER_H
#define OBJECTDETECTIONFILTER_H

#include "filter.h"

// Default standard deviation threshold
#define OBJECTDETECTION_STDDEV_THRES	30
#define OBJECTDETECTION_AVERAGE_THRES	200
//#define OBJECTDETECTION_DIFF_THRES		50

/**
 * ObjectDetectionFilter 
 */
class ObjectDetectionFilter : public Filter
{
    Q_OBJECT

public:

    static const QString TAG;

    ObjectDetectionFilter(QObject *parent = 0);

    // Input Frame
    FILTER_SET_MACRO(Frame, const cv::Mat&)

    FILTER_SET_MACRO(StdThreshold, int)
    FILTER_SET_MACRO(AverageThreshold, int)
	
    // Object detected flag
    FILTER_GET_MACRO(ObjectDetected, bool)

public slots:
    void process();
    void reset();

signals:
	void objectEnters();
	void objectLeaves();

private:
    bool ObjectDetected;
	
	int StdThreshold;
	int AverageThreshold;
	//int DiffThreshold;
	
    cv::Mat Frame;
};

#endif // OBJECTDETECTIONFILTER_H
