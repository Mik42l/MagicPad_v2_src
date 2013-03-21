#ifndef PIDFILTER_H
#define PIDFILTER_H

#include "filter.h"

// Default PID coefficient values
#define PIDFILTER_PROPORTIONAL 1.0
#define PIDFILTER_INTEGRAL 0.0
#define PIDFILTER_DERIVATIVE 0.0

// PID sample period in seconds
#define PIDFILTER_PERIOD 0.05

/**
 * PidFilter :
 * Template type must have the following features:
 * -> 'plus' and 'minus' operator. ie: T1+T2 and T1-T2 supported
 * -> 'times' double operator. ie: T*0.1 supported
 */
template <class T> class PidFilter : public Filter
{
public:
    static const QString TAG;

    PidFilter(QObject *parent = 0)
    {
        reset();
    }

    // Input
    void setInput(const T& _arg)
    {

        mError = _arg - Output;
        //Input = _arg;
    }
	
    FILTER_SET_MACRO(Proportional, double)
    FILTER_SET_MACRO(Integral, double)
    FILTER_SET_MACRO(Derivative, double)
    FILTER_SET_MACRO(Period, double)
	
    // Output Joystick position
    FILTER_GET_MACRO(Output, const T&)

public slots:
    void process()
    {
        mIntegral += mError * Period;

        T derivative = (mError - mLastError) * (1.0/Period);
        mLastError = mError;

        Output = Proportional*mError + Integral*mIntegral + Derivative*derivative;
    }

    void reset()
    {
        Proportional = PIDFILTER_PROPORTIONAL;
        Integral = PIDFILTER_INTEGRAL;
        Derivative = PIDFILTER_DERIVATIVE;
        Period = PIDFILTER_PERIOD;
    }

    // set Input==Output
    void initialize(const T& _in)
    {
        //Input = _in;
        Output = _in;
        mError = _in - _in;
        mLastError = mError;
        mIntegral = mError;
    }

private:	
    //T Input;
    T Output;
	
	// PID coeff
	double Proportional;
	double Integral;
	double Derivative;
    double Period;

    // new error
    T mError;
    // Last error
    T mLastError;
    // Integral error
    T mIntegral;
};

#endif // JOYSTICKFILTER_H
