#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include "ball.h"
#include "barplayer.h"
#include "barIA.h"
#include "scoremanager.h"
#include "collisionmanager.h"
#include "IAmanager.h"
#include "prisme.h"
#include <QList>
#include <QLine>
#include <QGraphicsLineItem>
#include <QSet>
#include <QTimer>
#include <QGraphicsPolygonItem>

/**
 *
 */
class GraphicsScene : public QGraphicsScene
{
    Q_OBJECT

public:
    GraphicsScene(qreal x, qreal y, qreal width, qreal height);
    void setupScene();
    BarPlayer* getBarPlayer();
    void decomposition();
    void ballDisparition(Ball* b);    
    void verifyVitesse(Ball* b);

public slots:
    void actualiserScene();
    void newPoint(Ball* b);
    void createLine(Ball* b);
    void initBallBlanche();
    void refreshLine();
    void stopScene();

signals:
    void finPartie();

private:
    Ball *ball;
    Ball *ballRouge;
    Ball *ballJaune;
    Ball *ballVerte;
    Ball *ballDefault;
    BarPlayer *barPlayer;
    BarIA *barIA;
    ScoreManager *scoreManager;
    CollisionManager *collisionManager;
    IAManager *iaManager;
    Prisme *prisme;
    bool dispersion;
    QList<Ball *> ballCollection;
    QSet<QGraphicsLineItem *> lineItemCollection;
    int triPoint;
    QTimer *timer;
    QTimer *timer2;
    qreal vitesseTemps;
    int vitesseVx;
    int oldVitesseVx;
    int vitesseVy;
    QGraphicsPolygonItem *prismeForme;
};

#endif // GRAPHICSSCENE_H
