#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include "ball.h"
#include "barIA.h"
#include "barplayer.h"
#include "prisme.h"
#include <QGraphicsPixmapItem>

class CollisionManager : public QObject
{
    Q_OBJECT
public:
    CollisionManager(BarIA* NewbarIA,BarPlayer* NewBarPlayer, Prisme* NewPrisme);
    void testCollision(Ball* b);
    void calculerVecteurV2(int vx, int vy, double nx, double ny, int raquette);
    bool collision(QGraphicsPixmapItem* item, Ball* b);

private:
    BarIA* barIA;
    BarPlayer* barPlayer;
    bool gauche;
    bool droite;
    Prisme* prisme;
    double v2xIA;
    double v2yIA;
    double v2xBP;
    double v2yBP;
};

#endif // COLLISIONMANAGER_H
