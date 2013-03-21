#ifndef SWITCHFILTER_H
#define SWITCHFILTER_H

#include "filter.h"

#define SWITCH_DEFAULT_THRESHOLD_HIGH 200
#define SWITCH_DEFAULT_THRESHOLD_LOW 150

/**
 * SwitchFilter : detect a press ON/OFF motion
 * This filter emulate a classic push button.
 * A toogle event is emitted if the input signal goes
 * bellow the 'low-threshold' and then above the 'high threshold'.
 * This hysteresis provides some level of immunity against noise.
 * _________
 *          \                ______
 *           \              /
 *            \            /<--High threshold ( toggle() emitted NOW!)
 *             \          /
 * Low thres--> \        /
 *               \______/
 */
class SwitchFilter : public Filter
{
	Q_OBJECT
	
public:

    static const QString TAG;

    SwitchFilter(QObject *parent = 0);

    // Input signal
    FILTER_SET_MACRO(Input, double)

    // mean value of input image
    FILTER_GET_MACRO(Toggle, bool)

    // Set the Low threshold
    FILTER_SET_MACRO(ThresholdLow, unsigned char)

    // Set the high threshold
    FILTER_SET_MACRO(ThresholdHigh, unsigned char)
	
public slots:

    void process();
	
    void reset();

signals:

    void toggle();

private:

    // Flag set to 'one' true' when a toggle() is emitted.
    // Reset to 'false' the rest of the time
    bool Toggle;
	
    // Input signal value
	double Input;
	
    // Internal state
	enum { IDLE, LOADED } mState;

    // Low threshold
    unsigned char ThresholdLow;

    // High threshold
    unsigned char ThresholdHigh;
    
};

#endif // SWITCHFILTER_H
