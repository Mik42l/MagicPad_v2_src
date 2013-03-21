#include "collisionmanager.h"
#include "ball.h"
#include "barIA.h"
#include "barplayer.h"
#include "math.h"
#include "prisme.h"
#include <QDebug>
#include <QGraphicsPixmapItem>

#define PI 3.14159265
#define valueTest 19790

/**
 *
 */
CollisionManager::CollisionManager(BarIA *NewbarIA, BarPlayer* NewBarPlayer, Prisme* NewPrisme) :
    barIA(NewbarIA),
    barPlayer(NewBarPlayer),
    prisme(NewPrisme),
    v2xIA(0.0),
    v2yIA(0.0),
    v2xBP(0.0),
    v2yBP(0.0)
{

}

/**
 *
 */
void CollisionManager::testCollision(Ball *b)
{
    if (collision(barPlayer,b)==true&&b->getInsideBP()==false)
    {
        int xo=barPlayer->x()+barPlayer->getPixmap()->width()/2;
        int yo=barPlayer->y()+barPlayer->getPixmap()->height()/2;
        int xe=b->getEndPoint()->x();
        int ye=b->getEndPoint()->y();
        int vx=b->getVx();
        int vy=b->getVy();

        double nx=xe-xo;
        double ny=ye-yo;
        double norme=sqrt(nx*nx+ny*ny);
        nx/=norme;
        ny/=norme;
        calculerVecteurV2(vx,vy,nx,ny,0);

        int intvx2= round(v2xBP);
        int intvy2= round(v2yBP);
        if(intvx2==0){intvx2=1;}

        b->setVx(intvx2);
        b->setVy(intvy2);
        b->setInsideBP(true);
        b->setCollision();
        }


    if (collision(barIA,b)==true&&b->getInsideBIA()==false)
    {
    int xo=barIA->x()+barIA->getPixmap()->width()/2;
    int yo=barIA->y()+barIA->getPixmap()->height()/2;
    int xe=b->getEndPoint()->x();
    int ye=b->getEndPoint()->y();
    int vx=b->getVx();
    int vy=b->getVy();

    double nx=xe-xo;
    double ny=ye-yo;
    double norme=sqrt(nx*nx+ny*ny);
    nx/=norme;
    ny/=norme;
    calculerVecteurV2(vx,vy,nx,ny,1);

    int intvx2= round(v2xIA);
    int intvy2= round(v2yIA);
    if(intvx2==0){intvx2=-1;}

    b->setVx(intvx2);
    b->setVy(intvy2);
    b->setInsideBIA(true);
    b->setCollision();
    }

    if (collision(barPlayer,b)==false&&b->getInsideBP()){b->setInsideBP(false);b->resetInsideBPCompteur();}
    if (collision(barIA,b)==false&&b->getInsideBIA()){b->setInsideBIA(false); b->resetInsideBIACompteur();}

    if(collision(barPlayer,b)&&b->getInsideBP())
    {
        b->incrementInsideBPCompteur();
        if(b->getInsideBPCompteur()>1)
        {
            b->setVx(10);
            b->setCollision();
        }
    }
    if(collision(barIA,b)&&b->getInsideBIA())
    {
        b->incrementeInsideBIACompteur();
        if(b->getInsideBIACompteur()>1)
        {
            b->setVx(-10);
            b->setCollision();
        }
    }

}


/**
 *
 */
bool CollisionManager::collision(QGraphicsPixmapItem *item, Ball *b)
{
    int d2 = (b->getEndPoint()->x()-item->x()-item->pixmap().width()/2)*(b->getEndPoint()->x()-item->x()-item->pixmap().width()/2)+(b->getEndPoint()->y()-item->y()-item->pixmap().height()/2)*(b->getEndPoint()->y()-item->y()-item->pixmap().height()/2);
    if(d2 > valueTest)
        return false;
    else
        return true;
}

/**
 *
 */
void CollisionManager::calculerVecteurV2(int vx,int vy, double nx, double ny,int raquette)
{
    if( raquette == 1 )
    {
        double pscal = (vx * nx + vy * ny);
        v2xIA = vx - 2 * pscal * nx;
        v2yIA = vy - 2 * pscal * ny;
    }

    if( raquette == 0 )
    {
        double pscal = ( vx * nx +  vy * ny );
        v2xBP = vx - 2 * pscal * nx;
        v2yBP = vy - 2 * pscal * ny;
    }
}
