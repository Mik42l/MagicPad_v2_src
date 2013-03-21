#include "switchFilter.h"

const QString SwitchFilter::TAG = QString("SwitchFilter");

/**
 *
 */
SwitchFilter::SwitchFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

/**
 *
 */
void SwitchFilter::process()
{
	Toggle = false;

	if( Input < SWITCH_DEFAULT_THRESHOLD_LOW && mState != LOADED )
	{
		mState = LOADED;
	}
	else if( Input > SWITCH_DEFAULT_THRESHOLD_HIGH && mState == LOADED )
	{
		Toggle = true;
        emit toggle();
		mState = IDLE;
	}
}

/**
 *
 */
void SwitchFilter::reset()
{
	mState = IDLE;

    Toggle = false;

    ThresholdLow = SWITCH_DEFAULT_THRESHOLD_LOW;

    ThresholdLow = SWITCH_DEFAULT_THRESHOLD_HIGH;
}
