#ifndef QUADRANTFILTER_H
#define QUADRANTFILTER_H

#include "filter.h"

#define QUADRANTFILTER_THRESHOLD 0.2

/**
 * QuadrantFilter
 */
class QuadrantFilter : public Filter
{
    Q_OBJECT

public:

    static const QString TAG;

    enum {
        QuadrantNone = 0,
        QuadrantTopLeft = 1,
        QuadrantTopRight = 2,
        QuadrantBottomLeft = 3,
        QuadrantBottomRight = 4
    } QuadrantPosition;

    QuadrantFilter( QObject *parent = 0 );

    // Vertical position:
    // -1.0: top
    // +1.0: bottom
    //FILTER_SET_MACRO(X, double)

    // Horizontal position:
    // -1.0: left
    // +1.0: right
    //FILTER_SET_MACRO(Y, double)

    FILTER_SET_MACRO(Position, const cv::Point2f&)

    // Hysteresis threshold width:
    //[-1 ... -Threshold/2 ... Threshold/2 ... +1]
    //            |                 |
    //  Right-to-left threshold     |
    //                           Left-to-right threshold
    FILTER_SET_MACRO(Threshold, double)

    //
    FILTER_GET_MACRO(Quadrant, int)

public slots:

    void process();

    void reset();

signals:
    void quadrantChanged( int quadrant );

private:
    cv::Mat Frame;

    cv::Point2f Position;
    double Threshold;
    int Quadrant;
};

#endif // QUADRANTFILTER_H
