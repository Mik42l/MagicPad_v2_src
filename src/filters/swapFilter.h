#ifndef SWAPFILTER_H
#define SWAPFILTER_H

#include "filter.h"

#include <QStateMachine>
#include <QTimer>

#define SWAPFILTER_THRESHOLD_LOW 190
#define SWAPFILTER_THRESHOLD_HIGH 210

/**
 * SwapFilter
 */
class SwapFilter : public Filter
{
    Q_OBJECT

public:

    static const QString TAG;

    SwapFilter( QObject *parent = 0 );

    FILTER_SET_MACRO(Frame, const cv::Mat&)

    FILTER_GET_MACRO(LeftToRightSwap, bool)
    FILTER_GET_MACRO(RightToLeftSwap, bool)

signals:
    void leftToRight(); // Process left->right movement
    void rightToLeft(); // Process right->left movement

signals:
    // Do not use directly
    void lowLeft();     // Left column goes low
    void highLeft();    // Left column goes high
    void lowRight();    // Right column goes low
    void highRight();   // Right column goes high
    void bothHigh();    // Both column goes high

public slots:

    void process();

    void reset();

    void l2r() { emit leftToRight(); LeftToRightSwap = true; }
    void r2l() { emit rightToLeft(); RightToLeftSwap = true; }

private:
    void setupStateMachine();

private:
    cv::Mat Frame;
    bool LeftToRightSwap;
    bool RightToLeftSwap;

    QStateMachine mFSM;    // Movement state machine
    unsigned char mThresHigh;  // Low->High threshold
    unsigned char mThresLow;   // High->Low threshold
};

#endif // SWAPFILTER_H
