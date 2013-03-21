#ifndef ARROWHINTWIDGET_H
#define ARROWHINTWIDGET_H

#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>
#include <QTimer>
#include <QWidget>

#include "QsLog.h"

/**
 * Display a left/right arrow in the top right corner
 * of the screen, on top of any running application.
 */
class ArrowHintWidget : public QWidget
{
    Q_OBJECT

public:

    const static QString TAG;

    typedef enum { ARROW_RIGHT, ARROW_LEFT } ArrowDirection;

    ArrowHintWidget(QWidget *parent = 0);

public slots:

    // Draw the widget
    void paintEvent(QPaintEvent *);

    // show the Arrow hint
    void popup(ArrowDirection direction);
    inline void popup() { popup( mDirection ); }
    
private:

    ArrowDirection mDirection;

};

#endif // ARROWHINTWIDGET_H
