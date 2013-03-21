#ifndef DEBOUNCEFILTER_H
#define DEBOUNCEFILTER_H

#include "filter.h"

#define DEBOUNCEFILTER_DEFAULT_WINDOW 10

/**
 * DebounceFilter
 */
template<class T>
class DebounceFilter : public Filter
{
public:

    static const QString TAG;

    DebounceFilter( QObject *parent = 0 ) : Filter( parent )
    {
        reset();
    }

    // Append a new sample at the end of the list
    void setNewSample( const T& sample )
    {
        mList.append( sample );

        while( mList.size() > WindowSize )
        {
            mList.pop_front();
        }
    }

    FILTER_SET_MACRO(WindowSize, int)

    // Is the input is Stable ?
    // A stable input means that all the last 'WindowSize' samples
    // are equals.
    FILTER_GET_MACRO(Stable, bool)

    // Return the most recent sample
    const T& getLastSample()
    {
        return mList.last();
    }

    //
    void process()
    {
        // check if all the values are equals
        if( ( mList.count( mList.last() ) == mList.size() )
            && ( mList.size() == WindowSize ) )
        {
            Stable = true;
        }
        else
        {
            Stable = false;
        }
    }

    void reset()
    {
        mList.clear();
        WindowSize = DEBOUNCEFILTER_DEFAULT_WINDOW;
    }

    void clear() { mList.clear(); }

private:
    int WindowSize;

    bool Stable;

    // Storage list
    QList< T > mList;
};

#endif // DEBOUNCEFILTER_H
