#include "appletButton.h"

/**
 *
 */
AppletButton::AppletButton(AppletInterface *applet, QGraphicsItem *parent)
    : QGraphicsWidget(parent)
{
    setAcceptHoverEvents(true);
    setCacheMode(DeviceCoordinateCache);
    setGraphicsEffect(new AppletShadowEffect());

    _pix = applet->icon();
    _name = applet->title();
    _applet = applet;
}

/**
 *
 */
QRectF AppletButton::boundingRect() const
{
    return QRectF(-_pix.width()/2, -_pix.height()/2, _pix.width(), _pix.height()+30);
}

/**
 *
 */
QPainterPath AppletButton::shape() const
{
    QPainterPath path;
    path.addEllipse(boundingRect());
    return path;
}

/**
 *
 */
void AppletButton::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    bool down = option->state & QStyle::State_Sunken;
    if(down) painter->translate(2, 2);

    // draw icon
    painter->drawPixmap(-_pix.width()/2, -_pix.height()/2, _pix);

    // draw text
    painter->setPen(Qt::white);
    QRect rect(-_pix.width()/2, _pix.height()/2, _pix.width(), 40);
    painter->drawText(rect, Qt::TextDontClip | Qt::AlignTop | Qt::AlignHCenter, _name);

    // shadow position
    QGraphicsDropShadowEffect *shadow = static_cast<QGraphicsDropShadowEffect *>(graphicsEffect());
    if(shadow != NULL)
    {
        if(down)
        {
            shadow->setOffset(APPLETSHADOW_X_OFFSET/2, APPLETSHADOW_Y_OFFSET/2);
            shadow->setBlurRadius(APPLETSHADOW_BLUR_RADIUS/2);
        }
        else
        {
            shadow->setOffset(APPLETSHADOW_X_OFFSET, APPLETSHADOW_Y_OFFSET);
            shadow->setBlurRadius(APPLETSHADOW_BLUR_RADIUS);
        }
    }
}

/**
 *
 */
void AppletButton::mousePressEvent(QGraphicsSceneMouseEvent *)
{
    update();
}

/**
 *
 */
void AppletButton::mouseReleaseEvent(QGraphicsSceneMouseEvent *)
{

    emit pressed(_applet);
    update();
}

/**
 *
 */
void AppletButton::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
   QGraphicsDropShadowEffect *shadow = static_cast<QGraphicsDropShadowEffect *>(graphicsEffect());
   Q_ASSERT(shadow);
}

/**
 *
 */
void AppletButton::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
   QGraphicsDropShadowEffect *shadow = static_cast<QGraphicsDropShadowEffect *>(graphicsEffect());
   Q_ASSERT(shadow);
   shadow->setColor(Qt::black);
}
