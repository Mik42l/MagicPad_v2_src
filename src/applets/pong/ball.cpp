#include "ball.h"

#include <QBrush>
#include <QGraphicsRectItem>
#include <QColor>
#include <QPen>
#include "math.h"

/**
 *
 */
Ball::Ball(int x, int y) : QGraphicsLineItem(),width(10),height(10),vx(10),vy(6)
  ,collision(false),direction(Ball::Down)
  ,startPoint(new QPoint(x,y)), endPoint(new QPoint(x,y)), arret(true),insideBP(false),insideBIA(false)
  ,insideBPCompteur(0),insideBIACompteur(0)
{
    QPen pen(Qt::white,3,Qt::SolidLine);
    this->setPen(pen);
    this->setZValue(2);
}

/**
 *
 */
int Ball::getWidth()
{
    return width;
}

/**
 *
 */
int Ball::getHeight()
{
    return height;
}

/**
 *
 */
void Ball::updatePosition()
{
    updateV();
    endPoint->setX( endPoint->x() + vx );
    endPoint->setY( endPoint->y() + vy );
    setLine( startPoint->x(), startPoint->y(), endPoint->x(), endPoint->y() );
}

/**
 *
 */
void Ball::updateV()
{
    if(this->endPoint->x()<=0){vx=-vx;emit but1(this);rebond();}
    if(this->endPoint->y()<=12){vy=fabs(vy);direction=Ball::Up;rebond();}
    if(this->endPoint->x()>=(976-this->width)){vx=-vx;emit but2(this);rebond();}
    if(this->endPoint->y()>=(660-this->height)){vy=-fabs(vy);direction=Ball::Down;rebond();}
    //trouver solution gestion collision via accès ici, ou dans graphicscene, ou via collisionmanager
}

/**
 *
 */
void Ball::setCollision()
{
    rebond();
}

/**
 *
 */
void Ball::resetOrientation(int newvx, int newvy)
{
    vx = newvx;
    vy = newvy;

    if(vy>0){
        direction = Ball::Down;
    } else {
        direction = Ball::Up;
    }
}

/**
 *
 */
enum Ball::Direction Ball::getDirection()
{
    return direction;
}

/**
 *
 */
QPoint* Ball::getEndPoint()
{
    return endPoint;
}

/**
 *
 */
QPoint* Ball::getStartPoint()
{
    return startPoint;
}

/**
 *
 */
void Ball::rebond()
{
    emit createLine( this );
    startPoint->setX( endPoint->x() );
    startPoint->setY( endPoint->y() );
}

/**
 *
 */
void Ball::resetPosition( int x, int y )
{
    startPoint->setX( x );
    startPoint->setY( y );
    endPoint->setX( x );
    endPoint->setY( y );
}

/**
 *
 */
int Ball::getVx()
{
    return vx;
}

/**
 *
 */
int Ball::getVy()
{
    return vy;
}

/**
 *
 */
void Ball::setVx(int Vx)
{
    vx = Vx;
}

/**
 *
 */
void Ball::setVy(int Vy)
{
    vy = Vy;
}

/**
 *
 */
void Ball::setStop(bool booleen)
{
    arret = booleen;
}

/**
 *
 */
bool Ball::isStop()
{
    return arret;
}

/**
 *
 */
void Ball::setInsideBIA(bool booleen)
{
    insideBIA = booleen;
}

/**
 *
 */
void Ball::setInsideBP(bool booleen)
{
    insideBP = booleen;
}

/**
 *
 */
bool Ball::getInsideBP()
{
    return insideBP;
}

/**
 *
 */
bool Ball::getInsideBIA()
{
    return insideBIA;
}

/**
 *
 */
void Ball::resetInside()
{
    insideBP = false;
    insideBIA = false;
}

/**
 *
 */
void Ball::incrementeInsideBIACompteur()
{
    insideBIACompteur++;
}

/**
 *
 */
void Ball::incrementInsideBPCompteur()
{
    insideBPCompteur++;
}

/**
 *
 */
int Ball::getInsideBIACompteur()
{
    return insideBIACompteur;
}

/**
 *
 */
int Ball::getInsideBPCompteur()
{
    return insideBPCompteur;
}

/**
 *
 */
void Ball::resetInsideBIACompteur()
{
    insideBIACompteur = 0;
}

/**
 *
 */
void Ball::resetInsideBPCompteur()
{
    insideBPCompteur = 0;
}
