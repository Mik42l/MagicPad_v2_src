#ifndef APPLETBUTTON_H
#define APPLETBUTTON_H

#include <QGraphicsDropShadowEffect>
#include <QGraphicsWidget>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

#include "applets/appletInterface.h"

#define APPLETSHADOW_BLUR_RADIUS 15.0
#define APPLETSHADOW_X_OFFSET 8.0
#define APPLETSHADOW_Y_OFFSET 8.0

class AppletShadowEffect : public QGraphicsDropShadowEffect
{
    Q_OBJECT
public:
    AppletShadowEffect(QObject *parent = 0) : QGraphicsDropShadowEffect(parent)
    {
        setBlurRadius(APPLETSHADOW_BLUR_RADIUS);
        setColor(Qt::black);
        setOffset(APPLETSHADOW_X_OFFSET, APPLETSHADOW_Y_OFFSET);
    }
};

class AppletButton : public QGraphicsWidget
{
    Q_OBJECT
public:
    AppletButton(AppletInterface *applet, QGraphicsItem *parent = 0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *);
    QPainterPath shape() const;

signals:
    void pressed();
    void pressed(AppletInterface *applet);

protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent *);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *);
    void mousePressEvent(QGraphicsSceneMouseEvent *);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *);

private:
    QPixmap _pix;
    QString _name;
    AppletInterface *_applet;
};


#endif // APPLETBUTTON_H
