#include "prisme.h"
#include <QPixmap>

/**
 *
 */
Prisme::Prisme() : QGraphicsPixmapItem()
{
    pixmap = new QPixmap( ":image/pong/prismeV4.png" );
    this->setPixmap(*pixmap);
    this->setZValue(5);
}

/**
 *
 */
QPixmap* Prisme::getPixmap()
{
    return pixmap;
}
