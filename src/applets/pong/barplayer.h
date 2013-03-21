#ifndef BARPLAYER_H
#define BARPLAYER_H

#include <QGraphicsRectItem>

/**
 *
 */
class BarPlayer : public QObject, public QGraphicsPixmapItem
{
   Q_OBJECT

public:
    BarPlayer();
    int getWidth();
    int getHeight();
    QPixmap* getPixmap();
    void testInitStop();

public slots:
    void modifyY(int y);

signals:
    void inactivite();

private:
    int width;
    int height;
    QPixmap* pixmap;
    int initStopCompteur;
    int lastPosY;
};

#endif // BARPLAYER_H
