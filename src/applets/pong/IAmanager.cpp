#include "IAmanager.h"
#include "barIA.h"
#include "ball.h"
#include <QDebug>


/**
 *
 */
IAManager::IAManager(BarIA *newBarIA):barIA(newBarIA)
{

}

/**
 *
 */
void IAManager::actualizeBarIADirection( const QList<Ball*> & ballCollection)
{
    Ball* proximityBall = ballCollection.last();
    int maxX = 0;

    foreach(Ball* b, ballCollection)
    {
        if(b->isStop()==false && b->getEndPoint()->x() > maxX)
        {
            maxX = b->getEndPoint()->x();
            proximityBall = b;
        }
    }

    try{
        if( proximityBall->getEndPoint()->x() > 760 &&
            proximityBall->getEndPoint()->y() > (barIA->pos().y()+barIA->getPixmap()->height()/2) )
        {
            barIA->setNextPosition(-barIA->getPixmap()->height()/2 + proximityBall->getEndPoint()->y());
        }
        else if(proximityBall->getEndPoint()->x() > 760 &&
                proximityBall->getEndPoint()->y() < (barIA->pos().y()+barIA->getPixmap()->height()/2))
        {
            barIA->setNextPosition(-barIA->getPixmap()->height()/2+proximityBall->getEndPoint()->y());
        }
    }catch(int){}
}
