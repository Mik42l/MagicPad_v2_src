#include "actionButton.h"

/**
 *
 */
ActionButton::ActionButton(const QPixmap& pix, QGraphicsItem *parent) :
    QGraphicsWidget(parent), mPixmap(pix)
{
    // internal config
    setAcceptHoverEvents(true);
    setCacheMode(DeviceCoordinateCache);

    // shadow effect
    setGraphicsEffect(new ButtonShadowEffect);

    mState = true;
    mCheckable = false;
}

/**
 *
 */
QRectF ActionButton::boundingRect() const
{
    return QRectF(0, 0, mPixmap.width(), mPixmap.height());
}

/**
 *
 */
QPainterPath ActionButton::shape() const
{
    QPainterPath path;
    path.addEllipse(boundingRect());
    return path;
}

/**
 *
 */
void ActionButton::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    bool pressed = option->state & QStyle::State_Sunken;
    bool hover = option->state & QStyle::State_MouseOver;

    if(pressed) painter->translate(1, 1);

    // draw icon
    painter->drawPixmap(0, 0, mPixmap);

    // shadow position
    QGraphicsDropShadowEffect *shadow = static_cast<QGraphicsDropShadowEffect *>(graphicsEffect());
    if(shadow != NULL)
    {
        if( mCheckable )
        {
            shadow->setBlurRadius( mState ? 20 : 7 );
        }
        else
        {
            if(hover) {
                shadow->setBlurRadius(20);
            } else {
                shadow->setBlurRadius(7);
            }
        }
    }
}

/**
 *
 */
void ActionButton::mousePressEvent(QGraphicsSceneMouseEvent *)
{
    update();
}
void ActionButton::mouseReleaseEvent(QGraphicsSceneMouseEvent *)
{
    emit pressed();
    if (!mState) emit pressedON();
    else emit pressedOFF();

    this->setState(!mState);

    update();
}

/**
 *
 */
void ActionButton::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
    update();
}
void ActionButton::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
    update();
}

/**
 *
 */
void ActionButton::setCheckable(bool checkable)
{
    mCheckable = checkable;
}

/**
 *
 */
void ActionButton::setPixmap(const QPixmap& pix)
{
    mPixmap = pix;
}

/**
 *
 */
void ActionButton::setState(const bool& s)
{
    mState = s;
}

/**
 *
 */
void ActionButton::setShadowColor(const QColor& color)
{
    QGraphicsDropShadowEffect *shadow = static_cast<QGraphicsDropShadowEffect *>(graphicsEffect());
    if(shadow != NULL)
    {
        shadow->setColor(color);
    }
}
