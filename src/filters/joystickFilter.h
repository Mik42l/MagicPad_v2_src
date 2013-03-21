#ifndef JOYSTICKFILTER_H
#define JOYSTICKFILTER_H

#include "filter.h"

/**
 * JoystickFilter 
 */
class JoystickFilter : public Filter
{
    Q_OBJECT
public:
    static const QString TAG;

    JoystickFilter(QObject *parent = 0);

    // Input Frame
    FILTER_SET_MACRO(Frame, const cv::Mat&)

    // Output Joystick position
    FILTER_GET_MACRO(Position, const cv::Point2f&)

public slots:
    void process();
    void reset();

signals:

private:	
    cv::Mat Frame;
    cv::Point2f Position;
};

#endif // JOYSTICKFILTER_H
