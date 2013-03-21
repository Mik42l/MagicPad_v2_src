#include "barplayer.h"
#include <QBrush>
#include <QGraphicsRectItem>
#include <QColor>
#include <QDebug>

/**
 *
 */
BarPlayer::BarPlayer() : QGraphicsPixmapItem() ,
    width(10),
    height(80),
    initStopCompteur(0),
    lastPosY(0)
{
    pixmap = new QPixmap( ":image/pong/raquette2.png" );
    setPixmap(*pixmap);
    setZValue(1);
}

/**
 *
 */
int BarPlayer::getWidth()
{
    return width;
}

/**
 *
 */
int BarPlayer::getHeight()
{
    return height;
}

/**
 *
 */
void BarPlayer::modifyY(int y)
{
    //config magic pad
    int posY = 672 - ( y * 672 ) / 255;
    posY = posY - getPixmap()->height()/2;
    setY( posY );
}

/**
 *
 */
QPixmap* BarPlayer::getPixmap()
{
    return pixmap;
}

/**
 *
 */
void BarPlayer::testInitStop()
{
    int actualPosY = pos().y();
    if( actualPosY == lastPosY ) {
        initStopCompteur += 1;
        /*qDebug()<<initStopCompteur;*/
    } else {
        initStopCompteur = 0;
    }

    if(initStopCompteur > 1000) {
        emit inactivite();
        /*qDebug()<<"inactivite";*/
    }

    lastPosY = actualPosY;
}
