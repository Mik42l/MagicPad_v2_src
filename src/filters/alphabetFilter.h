#ifndef ALPHABETFILTER_H
#define ALPHABETFILTER_H

#include "filter.h"

/**
 * AlphabetFilter 
 */
class AlphabetFilter : public Filter
{

	Q_OBJECT
	
public:

    static const QString TAG;

    AlphabetFilter(QObject *parent = 0);

    // Input Frame
    FILTER_SET_MACRO(Frame, const cv::Mat&)

    // mean value of input image
    FILTER_GET_MACRO(Letter, char)
	
public slots:

    void process();
	
    void reset();

protected slots:

private:

    char Letter;
	
    cv::Mat Frame;
};

#endif // ALPHABETFILTER_H
