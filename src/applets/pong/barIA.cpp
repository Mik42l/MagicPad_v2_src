#include "barIA.h"
#include <QBrush>
#include <QGraphicsRectItem>
#include <QColor>
#include "math.h"

/**
 *
 */
BarIA::BarIA() : QGraphicsPixmapItem(),
    width(10),
    height(80),
    vy(6),
    direction(BarIA::Down),
    nextPosition(0)
{
    setZValue(1);
    pixmap = new QPixmap( ":image/pong/raquette3.png" );
    setPixmap( *pixmap );
}

/**
 *
 */
int BarIA::getWidth()
{
    return width;
}

/**
 *
 */
int BarIA::getHeight()
{
    return height;
}

/**
 *
 */
void BarIA::updatePosition()
{
    setY( y() + ( nextPosition - y() ) * 0.20 );
}

/**
 *
 */
void BarIA::updateV()
{
    if( direction == BarIA::Up ) { nextPosition = y() - 10; }
    if( direction == BarIA::Down ){ nextPosition = y() + 10; }
    if( direction == BarIA::None ){}

}

/**
 *
 */
void BarIA::setDirection(Direction newDirection)
{
    direction = newDirection;
}

/**
 *
 */
QPixmap* BarIA::getPixmap()
{
    return pixmap;
}

/**
 *
 */
void BarIA::setNextPosition(int newNextPosition)
{
    nextPosition = newNextPosition;
}
