#include "graphicsscene.h"
#include "scoremanager.h"
#include "ball.h"
#include "barplayer.h"
#include "barIA.h"
#include <QBrush>
#include <QColor>
#include <QPen>
#include <QLine>
#include <QTimer>
#include "prisme.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QDebug>
#include <QGraphicsLineItem>
#include <QSet>
#include "math.h"
#include <QGraphicsPolygonItem>
#include <QPolygonF>
#include <QPointF>


/**
 *
 */
GraphicsScene::GraphicsScene(qreal x, qreal y,qreal width, qreal height):
    QGraphicsScene(x,y,width,height),ball(new Ball(500,500))
  ,ballRouge(new Ball(500,500)),ballJaune(new Ball(500,500)),ballVerte(new Ball(500,500)),ballDefault(new Ball(0,336))
  ,barPlayer(new BarPlayer)
  ,prismeForme(new QGraphicsPolygonItem())
  ,barIA(new BarIA),scoreManager(new ScoreManager),prisme(new Prisme)
  ,collisionManager(new CollisionManager(barIA,barPlayer,prisme))
  ,iaManager(new IAManager(barIA)),dispersion(false)
  ,triPoint(0)
  ,vitesseTemps(0)
  ,vitesseVx(0)
  ,oldVitesseVx(0)
  ,vitesseVy(0)

{

    setBackgroundBrush(QBrush(QColor("black")));
}

/**
 *
 */
void GraphicsScene::setupScene()
{
    ballCollection.append(ball);
    ballCollection.append(ballRouge);
    ballCollection.append(ballJaune);
    ballCollection.append(ballVerte);
    ballCollection.append(ballDefault);
    ballDefault->setStop(false);

    QPen pen2(Qt::white,1,Qt::SolidLine);
    QRectF rect(0,10,976,652);
    addRect(rect,pen2);

    prisme->setPos( width()/2-prisme->getPixmap()->width()/2, height()/2-prisme->getPixmap()->height()/2);
    addItem(prisme);

    //prismeForme->setPen(QPen(QBrush(QColor("red")),2,Qt::SolidLine));
    QPolygonF polygon;
    polygon << QPointF( width()/2,  height()/2-prisme->getPixmap()->height()/2) << QPointF( width()/2-prisme->getPixmap()->width()/2, this->height()/2+prisme->getPixmap()->height()/2)<<QPointF(this->width()/2+prisme->getPixmap()->width()/2,this->height()/2+prisme->getPixmap()->height()/2);
    prismeForme->setPolygon(polygon);
    addItem(prismeForme);

    barPlayer->setPos(-barPlayer->getPixmap()->width()/2,( height()/2)-(barPlayer->getPixmap()->height()/2));
    addItem(barPlayer);

    barIA->setPos( width()-barIA->getPixmap()->width()/2,( height()/2)-(barIA->getPixmap()->height()/2));
    addItem(barIA);

    scoreManager->getScoreView2()->setPos(670,560);
    addItem(scoreManager->getScoreView2());

    scoreManager->getScoreView1()->setPos(210,560);
    addItem(scoreManager->getScoreView1());

    initBallBlanche();

    timer = new QTimer( this );
    timer->setInterval( 20 );
    timer->start();
    connect( timer, SIGNAL(timeout()), this, SLOT(actualiserScene()) );

    timer2 = new QTimer(this);
    timer2->setInterval(1000);
    timer2->start();
    connect( timer2, SIGNAL(timeout()), this, SLOT(refreshLine()) );
    connect( barPlayer, SIGNAL(inactivite()), this, SLOT(stopScene()) );
    connect( this, SIGNAL(finPartie()), this, SLOT(stopScene()) );
}

/**
 *
 */
BarPlayer* GraphicsScene::getBarPlayer()
{
    return barPlayer;
}

/**
 *
 */
void GraphicsScene::actualiserScene()
{
    vitesseTemps += 1.0;
    vitesseVx = round( vitesseTemps / 500.0 );
    vitesseVy = round( vitesseTemps / 500.0 );

    if( !dispersion &&
        !ball->isStop() &&
        ball->getEndPoint()->x() > width()/4 &&
        ball->collidesWithItem( prismeForme )
        )
    {
        decomposition();
        dispersion = true;
    }

    foreach(Ball* newBall, ballCollection)
    {
        if(newBall != ballDefault && !newBall->isStop() )
        {
            collisionManager->testCollision( newBall );
            newBall->updatePosition();
            verifyVitesse( newBall );
        }
    }

    iaManager->actualizeBarIADirection( ballCollection );
    barIA->updatePosition();
    barPlayer->testInitStop();
}

/**
 *
 */
void GraphicsScene::newPoint(Ball *b)
{
    if(scoreManager->getScore1() >= 11 ||
        scoreManager->getScore2() >= 11) {
        emit finPartie();
    }

    if( b == ball )
    {
        b->resetOrientation( 10 + vitesseVx, 6 + vitesseVy );
        b->resetInside();
        b->resetPosition( barPlayer->x() + barPlayer->getPixmap()->width() + 5,
                          barPlayer->y() + barPlayer->getPixmap()->height()/2 );
    }
    else
    {
        ballDisparition(b);
        triPoint++;
    }

    if(triPoint==3)
    {
        dispersion = false;
        triPoint = 0;
        QTimer::singleShot( 1000, this, SLOT( initBallBlanche()) );
        //initBallBlanche();
    }
}

/**
 *
 */
void GraphicsScene::createLine(Ball *b)
{

    QLine line( b->getStartPoint()->x(),
                b->getStartPoint()->y(),
                b->getEndPoint()->x(),
                b->getEndPoint()->y()
               );

    QPen pen( b->pen().brush(), 3, Qt::SolidLine);

    QGraphicsLineItem* lineItem = new QGraphicsLineItem( line );
    lineItem->setPen( pen );
    lineItemCollection.insert( lineItem );
    addItem( lineItem );
}

/**
 *
 */
void GraphicsScene::decomposition()
{
    ballDisparition( ball );
    int i = 0;
    foreach( Ball* newBall, ballCollection )
    {
        if( newBall != ball && newBall != ballDefault )
        {
            newBall->resetPosition(ball->getEndPoint()->x(),ball->getEndPoint()->y());
            newBall->resetInside();

            int newvx = ball->getVx() - 1 + i;
            if( newvx == 0 ) {
                newvx = 1;
            }

            newBall->resetOrientation( newvx, ball->getVy() - 1 + i );
            addItem( newBall );
            newBall->setStop( false );

            connect( newBall, SIGNAL( but1(Ball*) ), scoreManager, SLOT( updateScore2() ) );
            connect( newBall, SIGNAL( but2(Ball*) ), scoreManager, SLOT( updateScore1() ) );
            connect( newBall, SIGNAL( but1(Ball*) ), this, SLOT( newPoint(Ball*) ) );
            connect( newBall, SIGNAL( but2(Ball*) ), this, SLOT( newPoint(Ball*) ) );
            connect( newBall, SIGNAL( createLine(Ball*) ), this, SLOT( createLine(Ball*) ) );
        }

        QPen pen( QBrush( QColor(231,47,31)) , 3 , Qt::SolidLine );
        ballRouge->setPen( pen );
        pen = QPen( QBrush( QColor(45,170,225)), 3, Qt::SolidLine );
        ballJaune->setPen( pen );
        pen = QPen( QBrush( QColor(82,172,49)), 3, Qt::SolidLine );
        ballVerte->setPen( pen );
        i++;
    }
}

/**
 *
 */
void GraphicsScene::ballDisparition(Ball* b)
{
    b->setStop( true );
    createLine( b );
    removeItem( b );
    disconnect( b, SIGNAL(but1(Ball*)), scoreManager, SLOT(updateScore2()));
    disconnect( b, SIGNAL(but2(Ball*)), scoreManager, SLOT(updateScore1()));
    disconnect( b, SIGNAL(but1(Ball*)), this, SLOT(newPoint(Ball*)));
    disconnect( b, SIGNAL(but2(Ball*)), this, SLOT(newPoint(Ball*)));
    disconnect( b, SIGNAL(createLine(Ball*)), this, SLOT(createLine(Ball*)));
}

/**
 *
 */
void GraphicsScene::initBallBlanche()
{
    ball->resetOrientation( 10 + vitesseVx, 6 + vitesseVy );
    ball->resetPosition( barPlayer->x() + barPlayer->getPixmap()->width() + 5,
                         barPlayer->y() + barPlayer->getPixmap()->height()/2 );
    ball->resetInside();

    connect( ball, SIGNAL(but1(Ball*)), scoreManager, SLOT(updateScore2()));
    connect( ball, SIGNAL(but2(Ball*)), scoreManager, SLOT(updateScore1()));
    connect( ball, SIGNAL(but1(Ball*)), this, SLOT(newPoint(Ball*)));
    connect( ball, SIGNAL(but2(Ball*)), this, SLOT(newPoint(Ball*)));
    connect( ball, SIGNAL(createLine(Ball*)), this, SLOT(createLine(Ball*)));
    ball->setStop(false);
    addItem(ball);
}

/**
 * Reduce alpha component of all the lines (ray of light)
 */
void GraphicsScene::refreshLine()
{
    foreach(QGraphicsLineItem* line, lineItemCollection)
    {
        double alpha = line->pen().color().alphaF() - 0.10;
        if( alpha <= 0 )
        {
            try{
                removeItem(line);
                lineItemCollection.remove(line);
            } catch(int){}
        }
        else
        {
            line->setPen( QPen( QBrush( QColor::fromRgbF( line->pen().color().redF(),
                                                          line->pen().color().greenF(),
                                                          line->pen().color().blueF(),
                                                          alpha)),
                                3,
                                Qt::SolidLine));
        }
    }
}

/**
 *
 */
void GraphicsScene::stopScene()
{
    disconnect( barPlayer, SIGNAL(inactivite()), this, SLOT(stopScene()) );
    disconnect( this, SIGNAL(finPartie()), this, SLOT(stopScene()) );
    timer->stop();
    timer2->stop();
}

/**
 *
 */
void GraphicsScene::verifyVitesse(Ball *b)
{
    if((b->getVx() * b->getVx() + b->getVy() * b->getVy()) < ((10+vitesseVx)*(10+vitesseVx) + (6+vitesseVy)*(6+vitesseVy)))
    {
        if(b->getVx() < 0) {
            b->setVx(b->getVx() - 1);
        } else {
            b->setVx(b->getVx() + 1);
        }

        if( b->getVy() < 0 ) {
            b->setVy( b->getVy() - 1 );
        } else {
            b->setVy( b->getVy() + 1 );
        }
    }
}
