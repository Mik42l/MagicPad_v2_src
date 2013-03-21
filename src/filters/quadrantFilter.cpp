#include "quadrantFilter.h"


const QString QuadrantFilter::TAG = QString("QuadrantFilter");

/**
 *
 */
QuadrantFilter::QuadrantFilter( QObject *parent ) :
    Filter( parent )
{
    reset();
}

/**
 *
 */
void QuadrantFilter::process()
{
    double _X = Position.x;
    double _Y = Position.y;

    // Top-Left
    if( (_X < -Threshold/2.0) && (_Y < -Threshold/2.0) && (Quadrant != QuadrantTopLeft) )
    {
        Quadrant = QuadrantTopLeft;
        emit quadrantChanged( Quadrant );
    }
    // Top-Right
    else if( (_X > Threshold/2.0) && (_Y < -Threshold/2.0) && (Quadrant != QuadrantTopRight) )
    {
        Quadrant = QuadrantTopRight;
        emit quadrantChanged( Quadrant );
    }
    // Bottom-Left
    else if( (_X < -Threshold/2.0) && (_Y > Threshold/2.0) && (Quadrant != QuadrantBottomLeft) )
    {
        Quadrant = QuadrantBottomLeft;
        emit quadrantChanged( Quadrant );
    }
    // Bottom-Right
    else if( (_X > Threshold/2.0) && (_Y > Threshold/2.0) && (Quadrant != QuadrantBottomRight) )
    {
        Quadrant = QuadrantBottomRight;
        emit quadrantChanged( Quadrant );
    }
}

/**
 *
 */
void QuadrantFilter::reset()
{
    Position.x = 0.0;
    Position.y = 0.0;
    Threshold = QUADRANTFILTER_THRESHOLD;
    Quadrant = QuadrantNone;
}
