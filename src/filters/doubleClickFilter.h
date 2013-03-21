#ifndef DOUBLECLICKFILTER_H
#define DOUBLECLICKFILTER_H

#include "filter.h"

// Default threshold for xcoor to trigger a detection event
#define DOUBLECLICK_DEFAULT_THRESHOLD 0.80

/**
 * 'DoubleClickFilter' detects double click hand gesture using
 * normalized cross-correlation and a list of reference pattern.
 *
 * When process() is called, the input stream is compared against
 * all the pattern in the pattern-list, if the maximum xcoor is
 * higher than the threshold (see setThreshold) a flag is set.
 *
 */
class DoubleClickFilter : public Filter
{

	Q_OBJECT
	
public:

    static const QString TAG;

    DoubleClickFilter(QObject *parent = 0);

    void addPattern( const cv::Mat& pattern );

    // Input signal
    FILTER_SET_MACRO(Input, double)

    // Detection threshold
    FILTER_SET_MACRO(Threshold, double)

    // Detection flag
    FILTER_GET_MACRO(Detected, bool)

    // Cross correlation at zero lag
    FILTER_GET_MACRO(XCorr, double)
	
public slots:

    void process();
	
    void reset();

signals:

    void detected();

private:

    double xcorr( const cv::Mat& pattern, const cv::Mat& stream );

    void loadDoubleClickPatterns();

private:

    double Input;

    bool Detected;

    double XCorr;

    // List of patterns to compair against
    QList< cv::Mat > mPatterns;

    // Maximum length of patterns
    int mPatternsMaxLenght;

    // Input stream of data
    cv::Mat mInputStream;
    
    double Threshold;

};

#endif // DOUBLECLICKFILTER_H
