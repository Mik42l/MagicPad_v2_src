#ifndef BALL_H
#define BALL_H

#include <QGraphicsRectItem>
#include <QPoint>

/**
 *
 */
class Ball : public QObject, public QGraphicsLineItem
{
    Q_OBJECT

public:

    enum Direction { Up, Down };

    Ball(int x, int y);
    int getWidth();
    int getHeight();
    void updateV();
    void updatePosition();
    void setStop(bool);
    bool isStop();


    void setCollision();
    void resetOrientation(int newvx, int newvy);
    void resetPosition(int x ,int y);
    void resetInside();
    enum Direction getDirection();
    QPoint* getEndPoint();
    QPoint* getStartPoint();
    void rebond();
    int getVx();
    int getVy();
    void setVx(int Vx);
    void setVy(int Vy);
    bool getInsideBP();
    bool getInsideBIA();
    void setInsideBP(bool);
    void setInsideBIA(bool);
    void incrementeInsideBIACompteur();
    void incrementInsideBPCompteur();
    int getInsideBIACompteur();
    int getInsideBPCompteur();
    void resetInsideBIACompteur();
    void resetInsideBPCompteur();

signals:
    void but1(Ball* b);
    void but2(Ball* b);
    void createLine(Ball* b);

private:
    int width;
    int height;
    int vx;
    int vy;
    QPoint* startPoint;
    QPoint* endPoint;
    bool collision;
    Direction direction;
    bool arret;
    bool insideBP;
    bool insideBIA;
    int insideBPCompteur;
    int insideBIACompteur;

};

#endif // BALL_H
