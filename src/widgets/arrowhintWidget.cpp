#include "arrowhintWidget.h"

const QString ArrowHintWidget::TAG = "ArrowHintWidget";

/**
 *
 */
ArrowHintWidget::ArrowHintWidget(QWidget *parent) :
    QWidget(parent, Qt::SplashScreen | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint )
{
    // UI
    resize( 100, 100 );
    hide();
    //setWindowFlags( Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint );
    setAttribute( Qt::WA_TranslucentBackground ); // transparent background
    setAttribute( Qt::WA_ShowWithoutActivating ); // do not steal focus
    setFocusPolicy( Qt::NoFocus );

    // POSITION: middle of the left side
    QRect screen = QApplication::desktop()->geometry();
    move( screen.topRight() - QPoint( width(), 0 ) );
}

/**
 *
 */
void ArrowHintWidget::paintEvent(QPaintEvent *)
{
    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing );

    QRadialGradient gradient( rect().center(), 50 );
    gradient.setColorAt( 0.0, QColor(255, 255, 255, 200) );
    gradient.setColorAt( 1.0, QColor(255, 255, 255,   0) );
    painter.fillRect( rect(), gradient );

    QPen pen( Qt::black );
    pen.setCapStyle( Qt::RoundCap );
    pen.setWidth( 10 );
    painter.setPen( pen );
    painter.drawLine( 20, 50, 80, 50 );

    if( mDirection == ARROW_LEFT )
    {
        painter.drawLine( 65, 30, 80, 50 );
        painter.drawLine( 65, 70, 80, 50 );
    }
    else
    {
        painter.drawLine( 100-65, 30, 100-80, 50 );
        painter.drawLine( 100-65, 70, 100-80, 50 );
    }
}

/**
 *
 */
void ArrowHintWidget::popup(ArrowDirection direction)
{
    mDirection = direction;

    QTimer::singleShot( 1000, this, SLOT( hide() ) );

    show();
}
