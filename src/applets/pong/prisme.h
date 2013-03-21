#ifndef PRISME_H
#define PRISME_H

#include <QGraphicsPixmapItem>
#include <QPixmap>

/**
 *
 */
class Prisme: public QGraphicsPixmapItem
{
public:
    Prisme();
    QPixmap* getPixmap();
private:
    QPixmap* pixmap;
};

#endif // PRISME_H
