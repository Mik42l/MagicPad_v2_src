#ifndef BARIA_H
#define BARIA_H

#include <QGraphicsPixmapItem>

/**
 *
 */
class BarIA : public QGraphicsPixmapItem
{

public:

    enum Direction{Up,Down,None};
    BarIA();
    int getWidth();
    int getHeight();
    void updatePosition();
    void updateV();
    void setDirection(Direction newDirection);
    QPixmap* getPixmap();
    void setNextPosition(int newNextPosition);

private:

    int width;
    int height;
    int vy;
    Direction direction;
    QPixmap* pixmap;
    int nextPosition;

};

#endif // BARIA_H
