#ifndef DELAYFILTER_H
#define DELAYFILTER_H

#include "filter.h"

// Default delay length
#define DELAYFILTER_DEFAULT_WINDOW 10

/**
 * DelayFilter implements a delay line. New samples are added
 * to the filter using setInput(T). The delay length is set using
 * setDelay(int). The output is retrived with the getOutput()
 * method and returns the (current-Delay) sample.
 *
 * \usage
 * DelatFilter<int> delay(this)
 * delay.setInput(7);
 * delay.setInput(6);     // -3
 * delay.setInput(2);     // -2
 * delay.setInput(-1);    // -1
 * delay.setInput(4);     // HEAD
 * std::cout << delay.getOutput(); // print '6'
 */
template<class T>
class DelayFilter : public Filter
{

public:

    static const QString TAG;

    DelayFilter( QObject *parent = 0 ) : Filter( parent )
    {
        reset();
    }

    // Append a new value to the filter.
    void setInput( const T& value )
    {
        mList.append( value );

        while( mList.size() > Delay+1 )
        {
            mList.pop_front();
        }
    }

    // Clear any samples from the filter
    void clear()
    {
        mList.clear();
    }

    // Return the oldest sample. If the number of samples is
    // less than the filter delay length the oldest sample is
    // returned.
    T& getOutput()
    {
        return mList.first();
    }

    // Is the delay filter ready, ie: is there enough samples
    bool isOutputReady()
    {
        return mList.size() == ( Delay + 1 );
    }

    // Process input data.
    void process()
    {
        // Nothing to do
    }

    // Reset the filter to its original state
    void reset()
    {
        mList.clear();
        Delay = DELAYFILTER_DEFAULT_WINDOW;
    }

    // Set the delay duration (in number of samples)
    // A delay of '1' means that Output is one step behind last
    void setDelay(int delay)
    {
        Delay = delay;
    }

private:

    // Window size
    unsigned int Delay;

    // Storage list
    QList< T > mList;
};

#endif // DELAYFILTER_H
